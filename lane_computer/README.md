# Lane Computer Task
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

# Implementation Details

## Task 1 - Lane Processing

### Missing Lane Interpolation
When one boundary is missing, interpolates using 3.5m lateral offset from existing boundary.

### Trajectory Generation
Calculates centerline by averaging corresponding points from left/right boundaries.
Publishes as LINE_STRIP marker for visualization.

### Additional Note

- Found missing info, interpulation is needed - either by taking a look at the 
previous point or by adding a constant offset (3.5 meters approx). 
- Easy solution was implemented first.
- Better solution should be implemented later, which will be a more robust 
interpolation method.
- I'm not fully satisfied with my implementation as this is a bare-bone solution. 
I didn't have time to implement the more robust solution I wanted, due to 
time constraints as some of my time was 'wasted' on bringing up and 
installing ros, foxglove, etc. on my personal laptop, though I did that after 
the initial commit so you can use the git info for reference on the time it took.

## Task 2 - Distance Estimation Approaches

### Primary: Transform-based (`/assignment/travelled_distance`)
Uses tf2 transforms between `odom` and `base_link` frames to calculate euclidean distance.

**Implementation:**
- Timer-based updates at 10Hz
- 2D distance calculation (x,y only)
- Accumulates distance from position deltas
- Publishes to required topic per specification

**Advantages:**
- Accurate position tracking 
- Handles vehicle dynamics automatically
- Deterministic and robust

### Secondary: Velocity Integration (`/assignment/travelled_distance_vel`)
Integrates linear velocity from `/vehicle/twist` topic over time.

**Implementation:**
- Event-driven updates (100Hz from bag data)
- Calculates magnitude of linear velocity vector
- Integrates velocity × time for distance
- Validation/backup method

**Advantages:**
- Higher update frequency
- Independent of transform infrastructure
- Lower latency

### Production Considerations

Primary distance estimation uses transform-based approach for accuracy.
Velocity integration serves as validation method to detect sensor issues.
Both outputs enable cross-verification of odometry health.

### Additional Notes
Several ideas of how to implement this:
- Use `cmd_vel` topic to estimate distance over time (implemented).
- Use the transform between the vehicle's frame and a fixed frame 
(like `odom` and `base_link`) to calculate the distance travelled (implemented).
- Measure using the IMU info over time (`/vehicle/imu/data_raw`). 
- Measure using the GPS info over time (`/vehicle/gps/fix`).
- Measure using the acceleration info over time (`/vehicle/accel`).
- Measure using the average of all the above methods, but it is costly.
- Given wheels' radius (not given), we can average over the 4 wheels' rotation 
velocity to estimate the distance travelled.

From the approaches I implemented I'd summarize:
For production use: Transform-based approach due to accuracy and robustness. 
For real-time applications: Velocity integration for higher update rates. 
For validation: Run all (both in this case) simultaneously to detect sensor or odometry 
issues and during testing, should have 'ground truth' to compare with.

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