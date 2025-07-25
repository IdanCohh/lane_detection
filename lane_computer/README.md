# Lane Computer 
(this was written at the beggining of the project to order the tasks at hand)
This package contains a ROS 2 node that performs two main tasks:
1.  **Lane-Center Trajectory Generation**: Subscribes to lane boundary markers and publishes a trajectory representing the center of the lane.
2.  **Travelled Distance Estimation**: Estimates and publishes the total distance travelled by the vehicle.

## Build Instructions

To build the package, navigate to your ROS 2 workspace and run:

```bash
colcon build --packages-select lane_computer
```

## Run Instructions

First, source your workspace:

```bash
source install/setup.bash
```

Then, launch the node:

```bash
ros2 run lane_computer lane_computer_node
```

You will also need to play the provided ROS bag file in another terminal:

```bash
ros2 bag play <path_to_your_bag_file>
```

## Additional Note:
### Bag Info
```bash
[WARN] [1753458873.375399958] [rosbag2_storage_mcap]: no message indices found, falling back to reading in file order

Files:             my_dev_env/src/home_assignment_L4/info/home_assignment.mcap
Bag size:          93.4 MiB
Storage id:        mcap
Duration:          37.214203608s
Start:             Jan  6 2025 14:09:10.187212411 (1736165350.187212411)
End:               Jan  6 2025 14:09:47.401416019 (1736165387.401416019)
Messages:          11388
Topic information: Topic: /camera/front/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 209 | Serialization Format: cdr
                   Topic: /camera/front/image_compressed | Type: sensor_msgs/msg/CompressedImage | Count: 558 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 1717 | Serialization Format: cdr
                   Topic: /vehicle/accel | Type: geometry_msgs/msg/AccelStamped | Count: 209 | Serialization Format: cdr
                   Topic: /vis/lanes | Type: visualization_msgs/msg/MarkerArray | Count: 209 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 10 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 994 | Serialization Format: cdr
                   Topic: /vehicle/gps/fix | Type: sensor_msgs/msg/NavSatFix | Count: 37 | Serialization Format: cdr
                   Topic: /vehicle/imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 3722 | Serialization Format: cdr
                   Topic: /vehicle/twist | Type: geometry_msgs/msg/TwistStamped | Count: 3723 | Serialization Format: cdr
```
### Task 1:
- Found missing info, interpulation is needed - either by taking a look at the 
previous point or by adding a constant offset (3.5 meters approx). 
- Easy solution was implemented first.
- Better solution should be implemented later, which will be a more robust 
interpolation method.

### Task 2:
- Several ideas to implement this:
-- Take a look at the IMU info over time.
-- Use cmd_vel topic to estimate distance over time.
-- Use the transform between the vehicle's frame and a fixed frame 
(like `odom` and `base_link`) to calculate the distance travelled.