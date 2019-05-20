# core_rover
Provides control, sensing and management systems to run on board the rover.

Ensure you have base_station, nova_common, and webots_ros repositories in your catkin workspace as well, and that you have installed all of their dependencies.

If you want to use the Webots simulator for testing, clone the simulator repository as well.

# Dependencies

MUST HAVE Linux version 16.04 with ROS kinetic installed.
http://wiki.ros.org/ROS/Tutorials

sudo apt-get install libsdl2-dev
pip install Jetson.GPIO

Create a directory with path core_rover/include/lib/x86-64 and include the Phoenix-Linux-SocketCAN-Example/lib/x86-64 binaries from this link in there:
```
https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example/tree/master/lib/x86-64
```
For the IMU, install just the RTIMULib section with instructions following this link: 
https://www.jetsonhacks.com/2015/07/01/bosch-imu-under-ros-on-nvidia-jetson-tk1/

libRTIMULib.so must be copied and pasted from the RTIMULib/Linux/build/RTIMULib directory into catkin_ws/devel/lib directory


