
For information on how to use RosAria, see <http://wiki.ros.org/ROSARIA>,
especially <http://wiki.ros.org/ROSARIA/Tutorials/How to use ROSARIA>.

## Usage

### Install ARIA C++ library 

* Install pre-built Aria package (you can also download this manually from [this link](https://web.archive.org/web/20181005213856/http:/robots.mobilerobots.com/wiki/ARIA#Download_Aria))

```
$ cd ~/Downloads
$ wget https://web.archive.org/web/20181005213856/http://robots.mobilerobots.com/ARIA/download/current/libaria_2.9.4+ubuntu16_amd64.deb
$ sudo dpkg -i libaria_2.9.4+ubuntu16_amd64.deb
```

### Build ROS 2 Packages 

* this repo contains custom msg and use that by itself. Build tutorial requried

```
$ cd ~/<your_ws>/src
$ git clone https://github.com/kimsooyoung/ros2aria.git
$ cd ../
$ colcon build --symlink-install --packages-select rosaria_msgs && source install/local_setup.bash
```

* check if custom message has built successfully

```
$ ros2 interface show rosaria_msgs/msg/BumperState 

std_msgs/Header header
bool[] front_bumpers
bool[] rear_bumpers
```

* Build ROS 2 Aria pkg

```
$ colcon build --symlink-install --packages-select rosaria
$ source install/local_setup.bash
```

### Run ROS2Aria Node

* Check serial port device id before execution (ex - /dev/ttyUSB0)

```
$ sudo chmod 777 /dev/ttyUSB0
$ ros2 run rosaria RosAria
```

[] ubuntu 22.04 check and create humble branch