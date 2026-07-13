# vio_node

`vio_node` is a learning-focused stereo visual-inertial odometry (VIO) node for
ROS 2. It is under active development and does not yet publish an odometry
estimate.

The current implementation rectifies stereo images, detects and tracks sparse
features, and calculates metric depth from stereo disparity. The remaining work
is to estimate visual motion, process the IMU stream, fuse the measurements, and
publish a state estimate.

## Frame Contract

| Symbol | ROS frame | Purpose |
| --- | --- | --- |
| `W` | `vio_odom` | Local, gravity-aligned VIO world frame |
| `I` | `chassis_imu` | IMU frame whose pose the estimator maintains |
| `B` | `base_link` | Vehicle body frame used in ROS output |
| `CL` | `front_stereo_camera_left_rgb` | Left camera and stereo reference frame |
| `CR` | `front_stereo_camera_right_rgb` | Right camera frame |

The existing `odom -> base_link` transform is ground truth. It is used only for
external evaluation and must not be consumed by the VIO estimator.

The VIO node will publish an independent estimate in `vio_odom`. It must not
publish `odom -> base_link`, because that transform already has an owner. The
`publish_tf` parameter therefore defaults to `false`.

### Transform Notation

`T_A_B` transforms coordinates from frame `B` into frame `A`:

```text
p_A = T_A_B * p_B
```

TF2 uses the same target/source relationship:

```cpp
lookupTransform(target_frame, source_frame, time)
```

For example, this lookup returns `T_I_CL`:

```cpp
lookupTransform(
    "chassis_imu",
    "front_stereo_camera_left_rgb",
    tf2::TimePointZero);
```

TF2 can compose this transform through the common `base_link` ancestor:

```text
T_I_CL = T_I_B * T_B_CL
```

The fixed transforms required by the estimator are:

- `T_I_CL`: left camera into IMU
- `T_I_CR`: right camera into IMU
- `T_I_B`: body into IMU

These transforms should be retrieved after TF becomes available and then cached,
because the sensor mounting geometry does not change at runtime.

### Camera Frame Validation

The image and `CameraInfo` messages use the configured `_rgb` frame IDs. Runtime
TF measurements confirm that these frames follow the optical convention used by
the camera projection model:

```text
+x right
+y down
+z forward
```

For the front-facing cameras, camera `+z` maps to body `+x`, camera `+x` maps to
body `-y`, and camera `+y` maps to body `-z`. The measured transform from the
right camera into the left camera is:

```text
translation = [0.150, 0.000, 0.000] m
rotation    = identity
```

Both `CameraInfo.r` matrices are identity, so the rectified feature coordinates
and physical `_rgb` optical frames have the same orientation. No additional
RGB-to-optical or rectification rotation is currently required.

The node treats frame IDs as part of the sensor contract. IMU, image, and
`CameraInfo` messages with unexpected frame IDs are rejected.

## Estimator State

The estimator will maintain the IMU state rather than the `base_link` state:

```text
p_WI  IMU position in the VIO world
q_WI  IMU orientation in the VIO world
v_WI  IMU velocity in the VIO world
b_g   gyroscope bias
b_a   accelerometer bias
```

The body pose for ROS output is derived from the fixed body-to-IMU extrinsic:

```text
T_WB = T_WI * T_I_B
```

## Timestamp Contract

Each synchronized stereo pair defines one visual measurement time `t_k`. The raw
camera time is the average of the left and right image timestamps:

```text
t_camera = (t_left + t_right) / 2
t_k = t_camera + camera_imu_time_offset_sec
```

A positive time offset moves the visual measurement later relative to the IMU
clock. The initial configured offset is zero.

For each visual frame, the estimator will eventually:

1. Integrate every IMU measurement over `(t_(k-1), t_k]`.
2. Interpolate IMU measurements at the integration boundaries when necessary.
3. Apply the visual correction at `t_k`.

Timestamp rules:

- Use sensor `header.stamp`; never substitute the node's current time.
- Require camera and IMU timestamps to use the same ROS clock.
- Reject duplicate or backward timestamps.
- Detect and report large IMU gaps.
- Reject stereo pairs whose timestamps exceed the configured synchronization
  tolerance.

## Initialization Contract

VIO initializes independently of ground truth:

- Define `vio_odom` at the first successful initialization.
- Set initial position to zero.
- Estimate gravity direction and gyroscope bias during a stationary interval.
- Set initial yaw to zero because gravity does not determine yaw.
- Bootstrap velocity using subsequent visual motion.
- Do not use wheel or ground-truth odometry as a hidden prior.

## Output Contract

The eventual `nav_msgs/msg/Odometry` output will use:

```text
header.frame_id = vio_odom
child_frame_id  = base_link
pose            = T_WB
twist           = body velocity expressed in base_link
header.stamp    = estimator state timestamp
```

Pose and twist covariance will come from the estimator rather than fixed values.
TF publication remains disabled while the ground-truth TF tree owns `base_link`.

## Development Roadmap

1. Define and enforce the frame, transform, timestamp, initialization, and output
   contracts.
2. Make stereo feature tracks geometrically reliable and estimator-ready.
3. Implement and validate stereo visual odometry.
4. Add timestamped IMU buffering, initialization, and preintegration.
5. Fuse visual and inertial measurements in an estimator backend.
6. Publish complete odometry, covariance, status, and optional TF output.
7. Add unit, rosbag replay, accuracy, failure-recovery, and performance tests.

### Point 1 Status: Complete

Completed:

- Added configurable world, body, IMU, and camera frame IDs.
- Established `vio_odom` as an independent VIO world.
- Removed ground-truth odometry from the estimator input path.
- Added explicit TF ownership and camera/IMU time-offset parameters.
- Initialized the TF2 buffer and listener.
- Retrieved and cached `T_I_CL`, `T_I_CR`, and `T_I_B`.
- Verified the camera optical axes, identity rectification, and horizontal
  `0.150 m` stereo geometry.
- Added strict frame-ID validation for IMU, image, and `CameraInfo` messages.
- Gated stereo processing on complete rectification, stereo calibration, and
  extrinsic readiness.
- Validated transform values and verified that the TF stereo baseline agrees
  with the projection-matrix baseline.
- Constructed each visual timestamp from the stereo midpoint and configured
  camera/IMU offset.
- Rejected duplicate or backward IMU and visual timestamps.

### Current Point 2 Status

Point 2 prepares stereo tracks for visual odometry. A surviving
`TrackedFeature` represents the same scene point in two consecutive stereo
frames:

```text
px_left_prev          left pixel at t_(k-1)
point_left_cam_prev   3D point in the left camera at t_(k-1)
px_left_curr          left pixel at t_k
px_right_curr         right pixel at t_k
point_left_cam_curr   3D point in the left camera at t_k
```

`age` is the number of consecutive frames in which the track has a valid
stereo observation. A new track starts at `1`; a track that survives temporal
tracking and current-frame stereo validation increments by one. Motion
estimation must only use tracks with `age >= 2`.

Planned work:

1. Correct and document feature-observation state advancement.
2. Add temporal forward-backward tracking validation.
3. Add stereo left-right consistency validation.
4. Improve spatial distribution, configurable thresholds, and diagnostics.
5. Add focused tracker tests.
