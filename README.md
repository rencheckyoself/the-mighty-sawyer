# the-mighty-sawyer

# Introduction
> What you're looking at is a ghost, once alive but now deceased. Once upon a time, it was a cornhole stadium that housed a professional cornhole league club known as the Evanston Zephyrs. Now it houses nothing but memories and a wind that stirs in the high grass of what was once an outfield, a wind that sometimes bears a faint, ghostly resemblance to the roar of a crowd that once sat here. We're back in time now, when the Evanston Zephyrs were still a part of the American League, and this mausoleum of memories was an honest-to-Pete stadium. But since this is strictly a story of make believe, it has to start this way: once upon a time, in Evanston, Illinois, it was tryout day. And though he's not yet on the field, you're about to meet a most unusual fella, a one-armed robot player named The Mighty Sawyer. (adapted from the opening narration of the _The Twilight Zone_ episode ["The Mighty Casey"](https://en.wikipedia.org/wiki/The_Mighty_Casey))

## Project description
This was roughly a four week project that required manipulation, sensing, and human interaction using a [Rethink Robotics' Sawyer robot](https://www.rethinkrobotics.com/sawyer).  The theme, for all the course projects, was Recreational Robotics.  Our team of four chose [cornhole](https://en.wikipedia.org/wiki/Cornhole), a lawn game popular in North America and Europe.  We thought it was a perfect recreational activity to play with a 7 degree-of-freedom, (roughly) 4 foot robotic arm.

This project uses Sawyer's wrist camera and the `find_object_2d` package to detect and grasp beanbags; standard grippers, with a custom (engraved) glove, for securing the bag during throws; the `intera_interface` API to send joint commands for throwing; and [AprilTag](http://wiki.ros.org/apriltag_ros) ROS package to estimate cornhole board and bag poses.  Simple scorekeeping strategy was also implemented for Sawyer using these tools.

## Overview
This project included four major parts:
1. [Manipulation](#manipulation)
2. [Computer vision](#computer-vision)
3. [Throwing](#throwing)
4. [Human robot interaction](#human-robot-interaction)

In addition, the team developed a [simple state machine](#simple-state-machine) and some gazebo simulation capabilities mostly for testing offline.

# Package layout & submitted code:
```
├── CMakeLists.txt
├── config
│   ├── joints_cfg_sawyer.yaml
│   ├── settings.yaml
│   ├── tags.yaml
│   └── teams.yaml
├── doc
│   ├── conf.py
│   └── index.rst
├── images
│   ├── display
│   │   ├── angry.png
│   │   ├── blank_screen.png
│   │   ├── blue_screen.png
│   │   ├── flag.png
│   │   ├── happy.png
│   │   ├── red_screen.png
│   │   ├── sad.png
│   │   ├── sleepy.png
│   │   └── thumbs_up.png
│   └── vision
│       ├── 1.png
│       ├── 2.png
│       ├── 3.png
│       ├── 4.png
│       ├── 5.png
│       ├── 6.png
│       └── 7.png
├── launch
│   ├── apriltags.launch
│   ├── continuous_detection.launch
│   ├── grab_bag.launch
│   ├── play_cornhole.launch
│   └── sawyer_sim.launch
├── nodes
│   ├── apriltag_pose
│   ├── camera_converter
│   ├── grab_bag_server
│   ├── sawyer_main_client
│   ├── sawyer_movement_server
│   ├── traj_with_moveit
│   └── traj_with_sawyer
├── package.xml
├── README.md
├── rviz
│   └── web.rviz
├── scripts
│   ├── display_png.py
│   ├── set_exposure.py
│   ├── switch_camera.py
│   └── test_movement_server.py
├── setup.py
├── src
│   └── the_mighty_sawyer
│       ├── imagelib.py
│       ├── __init__.py
│       ├── sawyer_controller.py
│       ├── tms_helper_functions.py
├── srv
│   ├── EvaluateThrowResult.srv
│   ├── ExecuteThrow.srv
│   ├── GetPose.srv
│   ├── GrabBag.srv
│   ├── MoveToThrowPos.srv
│   ├── SawyerStates.srv
│   ├── TagPose.srv
│   └── WaitForBag.srv
└── test
```

# Instructions to run code
1. Set up workspace and clone repo:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ME495-EmbeddedSystems/the-mighty-sawyer.git
```
2. catkin and source
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
3. Launch the project
```
roslaunch the_mighty_sawyer play_cornhole.launch
```

# Manipulation

# Computer Vision

# Throwing

# Human robot interaction

# Simple state machine


# References
