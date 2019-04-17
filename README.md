# core_rover
Provides control, sensing and management systems to run on board the rover.

# Dependencies

MUST HAVE Linux version 16.04 with ROS kinetic installed.
http://wiki.ros.org/ROS/Tutorials

Ensure you have base_station, nova_common, and webots_ros repositories in your catkin workspace as well, and that you have installed all of their dependencies.

sudo apt-get install libsdl2-dev
sudo apt-get install ros-kinetic-robot-localization

Replace the core_rover/lib/x86-64 binaries with the Phoenix-Linux-SocketCAN-Example/lib/x86-64 binaries from this link (they have the same name but are actually different):
```
https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example/tree/master/lib/x86-64
```

If you want to use the Webots simulator for testing, clone the simulator repository as well
