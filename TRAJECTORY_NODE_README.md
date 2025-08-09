# TrajectoryNode

A ROS2 node that reads JSON trajectory files and sends them as service call requests to autonomous robots.

## Features

- **JSON Trajectory Loading**: Reads trajectory waypoints from JSON files
- **Service Client**: Sends trajectory data via ROS2 service calls (using `nav_msgs/srv/GetPlan`)
- **Path Publishing**: Publishes trajectory as `nav_msgs/msg/Path` for visualization
- **Flexible Configuration**: Configurable target service, frequency, and auto-send behavior
- **Manual Triggering**: Provides a service interface to manually trigger trajectory sending

## Usage

### 1. Basic Usage

```bash
# Build the package
colcon build --packages-select slam_ros_node

# Source the workspace
source install/setup.bash

# Run with default parameters
ros2 run slam_ros_node TrajectoryNode_exe
```

### 2. Using Launch File

```bash
# Run with example trajectory file
ros2 launch slam_ros_node trajectory_node.launch.py

# Run with custom trajectory file
ros2 launch slam_ros_node trajectory_node.launch.py trajectory_file:=/path/to/your/trajectory.json

# Run with auto-send enabled
ros2 launch slam_ros_node trajectory_node.launch.py auto_send:=true send_frequency:=0.5
```

### 3. Node Parameters

- `trajectory_file_path` (string): Path to the JSON trajectory file
- `target_service_name` (string, default: "/get_plan"): Name of the target service to call
- `send_frequency` (double, default: 1.0): Frequency to send trajectory when auto-send is enabled (Hz)
- `auto_send` (bool, default: false): Whether to automatically send trajectory periodically

### 4. Services and Topics

**Published Topics:**
- `/trajectory/path` (`nav_msgs/msg/Path`): Trajectory visualization path

**Service Clients:**
- `{target_service_name}` (`nav_msgs/srv/GetPlan`): Sends trajectory planning requests

**Service Servers:**
- `/send_trajectory` (`std_srvs/srv/Trigger`): Manually trigger trajectory sending

### 5. Manual Triggering

You can manually trigger trajectory sending using the service:

```bash
ros2 service call /send_trajectory std_srvs/srv/Trigger "{}"
```

## JSON Trajectory Format

The trajectory file should be a JSON file with the following structure:

```json
{
  "trajectory": [
    {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,         // optional, defaults to 0.0
      "yaw": 0.0,       // optional, defaults to 0.0 (radians)
      "timestamp": 0.0  // optional, defaults to 0.0
    },
    {
      "x": 1.0,
      "y": 1.0,
      "z": 0.0,
      "yaw": 1.57,
      "timestamp": 1.0
    }
    // ... more waypoints
  ],
  "metadata": {        // optional metadata section
    "description": "Example trajectory",
    "frame_id": "map",
    "total_points": 2,
    "duration_seconds": 1.0
  }
}
```

**Required Fields:**
- `x`, `y`: Position coordinates (meters)

**Optional Fields:**
- `z`: Height coordinate (meters, default: 0.0)
- `yaw`: Orientation in radians (default: 0.0)
- `timestamp`: Time associated with the waypoint (seconds, default: 0.0)

## Example Usage Scenarios

### Scenario 1: Autonomous Navigation
```bash
# Load a predefined patrol route and send it to the navigation stack
ros2 launch slam_ros_node trajectory_node.launch.py \
  trajectory_file:=/home/user/patrol_route.json \
  target_service:=/navigate_to_pose \
  auto_send:=true \
  send_frequency:=0.1
```

### Scenario 2: Manual Testing
```bash
# Load a test trajectory for manual verification
ros2 run slam_ros_node TrajectoryNode_exe \
  --ros-args \
  -p trajectory_file_path:=/home/user/test_trajectory.json \
  -p target_service_name:=/move_base/make_plan \
  -p auto_send:=false
```

Then trigger manually:
```bash
ros2 service call /send_trajectory std_srvs/srv/Trigger "{}"
```

### Scenario 3: Visualization Only
```bash
# Just publish the path for visualization without sending service calls
ros2 run slam_ros_node TrajectoryNode_exe \
  --ros-args \
  -p trajectory_file_path:=/home/user/visualization_path.json \
  -p target_service_name:=/non_existent_service \
  -p auto_send:=false
```

## Dependencies

- ROS2 (tested with Humble/Iron)
- nlohmann/json library
- Standard ROS2 message packages (nav_msgs, geometry_msgs, std_srvs)

## Building

Make sure you have the required dependencies installed:

```bash
# Install nlohmann-json
sudo apt install nlohmann-json3-dev

# Build the package
colcon build --packages-select slam_ros_node
```

## Troubleshooting

1. **"Service not available" warnings**: The target service is not running. Start your navigation/planning service or change the `target_service_name` parameter.

2. **"Failed to load trajectory"**: Check that the JSON file exists and has the correct format.

3. **Build errors related to nlohmann/json**: Install the nlohmann-json development package:
   ```bash
   sudo apt install nlohmann-json3-dev
   ```

4. **No trajectory visualization**: Make sure you're subscribing to `/trajectory/path` topic in your visualization tool (RViz2).
