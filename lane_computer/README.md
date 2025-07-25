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
