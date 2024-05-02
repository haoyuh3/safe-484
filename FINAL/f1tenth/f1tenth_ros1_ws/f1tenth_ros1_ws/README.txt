# ----------------------------------------------

$ catkin_make

$ source devel/setup.bash
$ roscore

$ source devel/setup.bash
$ python3 vicon_bridge.py

$ source devel/setup.bash
$ roslaunch racecar teleop.launch

$ source devel/setup.bash
$ roslaunch racecar sensors.launch /  
$ or roslaunch realsense2_camera rs_camera.launch

$ source devel/setup.bash
$ roslaunch racecar lidar.launch

# ----------------------------------------------

$ source devel/setup.bash
$ roslaunch racecar visualization.launch

