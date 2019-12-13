# the-mighty-sawyer

# Introduction
> What you're looking at is a ghost, once alive but now deceased. Once upon a time, it was a cornhole stadium that housed a professional cornhole league club known as the Evanston Zephyrs. Now it houses nothing but memories and a wind that stirs in the high grass of what was once an outfield, a wind that sometimes bears a faint, ghostly resemblance to the roar of a crowd that once sat here. We're back in time now, when the Evanston Zephyrs were still a part of the American League, and this mausoleum of memories was an honest-to-Pete stadium. But since this is strictly a story of make believe, it has to start this way: once upon a time, in Evanston, Illinois, it was tryout day. And though he's not yet on the field, you're about to meet a most unusual fella, a one-armed robot player named The Mighty Sawyer. (adapted from the opening narration of the _The Twilight Zone_ episode ["The Mighty Casey"](https://en.wikipedia.org/wiki/The_Mighty_Casey))

## Project description
This was roughly a four week project that required manipulation, sensing, and human interaction using a [Rethink Robotics' Sawyer robot](https://www.rethinkrobotics.com/sawyer).  The theme, for all the course projects, was Recreational Robotics.  Our team of four chose [cornhole](https://en.wikipedia.org/wiki/Cornhole), a lawn game popular in North America and Europe.  We thought it was a perfect recreational activity to play with a 7 degree-of-freedom, (roughly) 4 foot robotic arm.

This project uses Sawyer's wrist camera and the `find_object_2d` package to detect and grasp beanbags; standard L-grippers, with a custom (engraved) glove, for securing the bag during throws; the `intera_interface` API to send joint commands for throwing; and [AprilTag](http://wiki.ros.org/apriltag_ros) ROS package to estimate cornhole board and bag poses.  Simple scorekeeping strategy was also implemented for The Mighty Sawyer (TMS) using these tools.

## Overview
This project included three major parts:
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
│   ├── play_cornhole.launch - launch all nodes to play cornhole with TMS
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

# Computer Vision
In order to keep track of the cornhole board and bags positioned across the board, april tags are used as a means of identifying and extracting information of the board or bag's pose relative to the head camera. All april tags are part of the same tag family, 36h11, and each tag within this family is unique to allow Sawyer to distinguish tags between the board and all the different bags. These pose positions are used to assist in throwing and keeping track of score. Furthermore, detection of tags is designed in such a way that Sawyer will remember the last detected position of the tag if the tag is unable to be detected again. 

# Throwing
Since Sawyer is not the fastest robot, the throwing motion attempts to leverage as many joints as possible while also taking advantage of the full arm length. TMS executes an overhand throw, by positioning the arm behind it self. It then actuates joints 1, 3, and 5 to maximize velocity in the direction of the throw. During the trajectory, the grippers will open near the peak, releasing the bag. **Be sure that Sawyer's arm can be fully extended in all directions without hitting anything in the environment before running.**

TMS will also attempt to target the cornhole board and it will also make adjustments based on the result of each throw by referencing the respective April tags. These features are both accomplished using the distance and heading calculations between sawyer, the board, and the most recently thrown bag.

# Human robot interaction
TMS relies on a human player to show it a bag using the wrist camera and place it in the gripper when instructed by the screen. Sawyer will assume it is on the team of the first color bag you try to hand it. One it has a team set, it can detect when you hand it the wrong color bag.

It is also able to respond to a human player moving the board mid game. If TMS detects the board has shifted too far from the previously known position, it will display and angry emoji and retarget before the next throw.

# Simple state machine

0. Initialization
	1. TMS startup (e.g., robot enable, gripper calibration)
	2. Go to home position
	3. Initial targeting (e.g., find the cornhole board, make preliminary estimations)
	4. Start a new game (e.g., clear the scoreboard)
1. Grab bag
	1. Detect bag using wrist camera
	2. Check if the bag is correct (first turn initializes the team color)
	3. If correct, TMS waits for the human teammate to insert the beanbag between its grippers
	4. Check if the bag is inserted
	5. If inserted, grasp
2. Throw bag
	1. Go to throw position
	2. Execute throw
3. Update game state
	1. Make pose estimations of bags
	2. Make target/throw adjustments
	3. Determine how to map pose estimations to update the state of the game
	4. Update score
	5. Goto 1 (unless there is a winner)

```
[0: Initialization] -->  [1: Grab]  -->  [2: Throw]  -->  [3: Update]
							 ^									 |	
							 |-----------------------------------| 
```
TMS's high-level states are tracked within `sawyer_main_client` in the `nodes` folder.  Each main component of our project (e.g., robot motions) offer a suite of Services; these are then used in `sawyer_main_client` accordingly depending on the state of the cornhole game.  `sawyer_main_client` does not necessarily have access to all Services offered by the lower-level components, only those needed at the highest level to navigate through a game.  For instance, `actuate_gripper` Service is a low-level Service offered in `sawyer_movement_server` used for `grab_bag`.  Using only the necessary Services to play the game, TMS is able to transition between them until the game is over.  Though ROS Action Servers and SMACHs were considered as ways to implement TMS's state machine for this project, we viewed its complete state machine to be too simple.  

# Notes
## Lessons Learned
 - Throwing is a task so intuitive for a human that we rarely stop to realize the complexities of what is actually happening. The amount of hand control and sensory feedback a person has and can actually interpret far surpasses what a robot has access to. That being said, the great advantage a robot has over a person is the precision and repeatability.

## Future Work
 - The targeting features are currently restricted to the area to Sawyer's front left. For further development, we would ideally have the head camera scan until it found the board then select a throwing configuration based on that.
 - Also we would like to try to remove the dependency on AprilTags. This may prove to be too difficult for the bag detection with Sawyer's built in cameras, but the board should be possible.

# Media
Photos here.

# References
* [Sphinx API](https://rencheckyoself.github.io/the-mighty-sawyer/)
* [Final Video on YouTube](https://www.youtube.com/watch?v=GHv42RLQk-g&t=5s)
