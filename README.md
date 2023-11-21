# ROS 2 Beginner Tutorials - ENPM808X

This branch contains the publisher subscriber package for the ROS beginner tutorials phase of the ENPM808X couse, Week 9. Made by **Vinay Lanka** (12041665) as a part of the course *ENPM808X: Software Development for Robotics* at the University of Maryland.

### Dependencies
This project makes use of the ROS Humble Hawksbill distribution and is assumed to be a dependency. <br>
Find installation instructions [here](https://docs.ros.org/en/humble/Installation.html)

### Building the Code

```bash
$ source /opt/ros/humble/setup.bash
# Make your ros2 workspace
$ mkdir -p ~/ros_ws/src
# Go to the source directory of your ros2 workspace
$ cd ~/ros_ws/src
#Clone the repository
$ git clone git@github.com:vinay-lanka/beginner_tutorials.git
#Go back to the ws directory
$ cd ~/ros_ws
# Install rosdep dependencies before building the package
$ rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
$ colcon build --packages-select beginner_tutorials
# After successfull build source the package
$ source ./install/setup.bash

# Run the publisher in terminal#1
ros2 run beginner_tutorials talker
# Run the subscriber in terminal#2 (Split the terminal and source ROS2 and the workspace setup.bash)
ros2 run beginner_tutorials listener 
```

### Service - Change String
There's a service in the talker node to change the default string that get's published, to call it
```bash
$ ros2 service call /change_string beginner_tutorials/srv/ChangeStr "{new_string: New String}"
```

### Launch Files
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the publisher in terminal
$ ros2 launch beginner_tutorials talkernode.launch.py publish_freq:=1200 ros2_bag_start:=False
```

### TF Frames

The talker node in this package now publishes a static tf transform between 2 frames, `world` and `talk` as a arbitrary transform. To run the publisher, run
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the publisher in terminal
$ ros2 run beginner_tutorials talker
```
To view the tf transform, run the following commands in a separate terminal
```bash
 # In a new terminal window, echo the topic that broadcasts the static frame:
$ ros2 topic echo /tf_static
# In a new terminal window, get more information about the frames
$ ros2 run tf2_tools view_frames
```

### Testing
To run the unit tests and verify their working, run the commands below.
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
# Build the package:
$  colcon build --packages-select beginners_tutorials
# Install the package:
$  source install/setup.bash
# Run the unit tests:
$ colcon test --packages-select beginners_tutorials
# View the results pf the tests:
$ cat log/latest_test/beginners_tutorials/stdout_stderr.log
```

### ROS2 Bag Functionality
This package supports recording and playback of ros2 bags. The launch file has been modified to support ros2 bag recording. To record use the `ros2_bag_start` parameter (True/False).

```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the launch file in terminal with the ros2_bag_start parameter as true
$ ros2 launch beginner_tutorials talkernode.launch.py ros2_bag_start:=True
```
The above ros2 bag is called `talkerbag` and can be found in the workspace directory where the command was run.
To inspect and playback the ros2 bag.
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Inspect the ros2 bag
$  ros2 bag info talkerbag
# Play back the contents of the ros2 bag
$  ros2 bag play talkerbag
```
To check the working, in a seperate terminal run
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the listener in terminal
$ ros2 run beginner_tutorials listener
```

### Buidling Doxygen Documentation
```bash
$ cd ~/ros_ws
#Run the colcon build on the doxygen docs cmake target
$ colcon build --packages-select beginner_tutorials --cmake-target docs
```

### Check style guidelines
```bash
#In the package directory
cd ~/ros_ws/src/beginner_tutorials

# Cppcheck
$ cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" ) --check-config > results/cppcheck.txt

# cpplint
$ cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order  src/*.cpp >  results/cpplint.txt
```
