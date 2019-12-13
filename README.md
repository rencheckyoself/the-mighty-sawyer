# the-mighty-sawyer

# Introduction
> What you're looking at is a ghost, once alive but now deceased. Once upon a time, it was a cornhole stadium that housed a professional cornhole league club known as the Evanston Zephyrs. Now it houses nothing but memories and a wind that stirs in the high grass of what was once an outfield, a wind that sometimes bears a faint, ghostly resemblance to the roar of a crowd that once sat here. We're back in time now, when the Evanston Zephyrs were still a part of the American League, and this mausoleum of memories was an honest-to-Pete stadium. But since this is strictly a story of make believe, it has to start this way: once upon a time, in Evanston, Illinois, it was tryout day. And though he's not yet on the field, you're about to meet a most unusual fella, a one-armed robot player named The Mighty Sawyer. (adapted from the opening narration of the _The Twilight Zone_ episode ["The Mighty Casey"](https://en.wikipedia.org/wiki/The_Mighty_Casey))

## Project description
This was roughly a four week project that required manipulation, sensing, and human interaction using a [Rethink Robotics' Sawyer robot](https://www.rethinkrobotics.com/sawyer).  The theme, for all the course projects, was Recreational Robotics.  Our team of four chose [cornhole](https://en.wikipedia.org/wiki/Cornhole), a lawn game popular in North America and Europe.  We thought it was a perfect recreational activity to play with a 7 degree-of-freedom, (roughly) 4 foot robotic arm.

This project uses Sawyer's wrist camera and the `find_object_2d` package to detect and grasp beanbags; standard grippers, with a custom (engraved) glove, for securing the bag during throws; the `intera_interface` API to send joint commands for throwing; and [AprilTag](http://wiki.ros.org/apriltag_ros) ROS package to estimate cornhole board and bag poses.  Simple scorekeeping strategy was also implemented for Sawyer using these tools.

## Overview
This project included four major parts:
1. [Computer vision](#computer-vision)
2. [Throwing](#throwing)
3. [Human robot interaction](#human-robot-interaction)

In addition, the team developed a [simple state machine](#simple-state-machine) and some gazebo simulation capabilities mostly for testing offline.

# Package layout & submitted code:
```
├── CMakeLists.txt
├── config
│   ├── joints_cfg_sawyer.yaml
│   ├── settings.yaml
│   ├── tags.yaml
│   └── teams.yaml
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
│   ├── apriltags.launch - included continuous_detection.launch and launches rviz
│   ├── continuous_detection.launch - everything necessary for april tags
│   ├── grab_bag.launch - launch node containing services to grab a bag
│   ├── play_cornhole.launch - launch all nodes to play cornhole with sawyer
│   └── sawyer_sim.launch - launch node containing services to move the arm in simulation
├── nodes
│   ├── apriltag_pose - provides services to get the pose from april tags
│   ├── camera_converter - converts the camera image from Sawyer
│   ├── grab_bag_server - provides service to grab the bag from a player
│   ├── sawyer_main_client - main node containing the state machine
│   └── sawyer_movement_server - provides services to move the arm and target
├── package.xml
├── README.md
├── rviz
│   └── web.rviz
├── scripts
│   ├── display_png.py
│   ├── set_exposure.py
│   ├── switch_camera.py
│   └── test_movement_server.py - simple test script to test the movement services
├── setup.py
├── src
│   └── the_mighty_sawyer
│       ├── imagelib.py - library with camera related functions
│       ├── __init__.py
│       ├── sawyer_controller.py - library to move the arm
│       └── tms_helper_functions.py - general purpose helper functions
└── srv
    ├── EvaluateThrowResult.srv
    ├── ExecuteThrow.srv
    ├── GetPose.srv
    ├── GrabBag.srv
    ├── MoveToThrowPos.srv
    ├── SawyerStates.srv
    ├── TagPose.srv
    └── WaitForBag.srv

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

Note: Will need to take images of your own bean bags in order for the visual detection to work properly. Also will need to generate and add your own april tags.

# Notes
## Lessons Learned
 - Throwing is a task so intuitive for a human that we rarely stop to realize the complexities of what is actually happening. The amount of hand control and sensory feedback a person has and can actually interpret far surpasses what a robot has access to. That being said, the great advantage a robot has over a person is the precision and repeatability.

## Future Work
 - The targeting features are currently restricted to the area to Sawyer's front left. For further development, we would ideally have the head camera scan until it found the board then select a throwing configuration based on that.
 - Also we would like to try to remove the dependency on April tags. This may prove to be too difficult for the bag detection with Sawyer's built in cameras, but the board should be possible.

# Computer Vision

# Throwing
Since Sawyer is not the fastest robot, the throwing motion attempts to leverage as many joints as possible while also taking advantage of the full arm length. Sawyer executes an overhand throw, by positioning the arm behind it self. It then actuates joint 1, 3, and 5 to maximize velocity in the direction of the throw. During the trajectory, the grippers will open near the peak, releasing the bag. **Be sure that Sawyer's arm can be fully extended in all directions without hitting anything in the environment before running.**

Sawyer will also attempt to target the cornhole board and it will also make adjustments based on the result of each throw by referencing the respective April tags. These features are both accomplished using the distance and heading calculations between sawyer, the board, and the most recently thrown bag.

# Human robot interaction
Sawyer relies on a human player to show it a bag using the wrist camera and place it in the gripper when instructed by the screen. Sawyer will assume it is on the team of the first color bag you try to hand it. One it has a team set, it can detect when you hand it the wrong color bag.

It is also able to respond to a human player moving the board mid game. If Sawyer detects the board has shifted too far from the previously known position, it will display and angry emoji and retarget before the next throw.

# Simple state machine

# References

[Final Video on YouTube](https://www.youtube.com/watch?v=GHv42RLQk-g&t=5s)
