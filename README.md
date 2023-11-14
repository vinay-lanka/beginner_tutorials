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
$ ros2 launch beginner_tutorials talkernode.launch.py publish_freq:=1200
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
