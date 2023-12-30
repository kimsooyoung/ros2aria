
For information on how to use RosAria, see <http://wiki.ros.org/ROSARIA>,
especially <http://wiki.ros.org/ROSARIA/Tutorials/How to use ROSARIA>.

## Usage

1. this repo contains custom msg and use that by itself. Build tutorial requried

```
cd ~/<your_ws>/src
git clone https://github.com/kimsooyoung/ros2aria.git
cd ../
colcon build --symlink-install --packages-select rosaria_msgs && source install/local_setup.bash
```

| check if custom message has built successfully

```
ros2 interface show rosaria_msgs/msg/BumperState 
std_msgs/Header header
bool[] front_bumpers
bool[] rear_bumpers
```

2. Build ROS Aria pkg

```
colcon build --symlink-install --packages-select rosaria
source install/local_setup.bash
```

[] ubuntu 22.04 check and create humble branch