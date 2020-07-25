# ROS 2 Bumper2PC
Create Pointcloud from robot's bumper events.  
This is based on https://github.com/yujinrobot/kobuki/tree/devel/kobuki_bumper2pc  

## How it works
This ROS 2 node subscribes to a `/bumper` node which emits custom `Bumper` events coming from robot's hardware (needs to be implemented separately).  
Based on a bumper state it will output a Pointloud with a fake obstacle point which will make ROS 2 Navigation stack recalculate the route.  

## Build
```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select bumper2pc
```

## Run
```bash
source install/setup.bash
ros2 run bumper2pc bumper2pc
```

or
```bash
source install/setup.bash
ros2 launch bumper2pc bumper2pc_launch.py
```

## Publish a bumper message for testing

```bash
ros2 topic pub /bumper bumper2pc/msg/Bumper "{left: true, center: false, right: false}"
```