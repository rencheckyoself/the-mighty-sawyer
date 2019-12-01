# the-mighty-sawyer

# Project Overview
> What you're looking at is a ghost, once alive but now deceased. Once upon a time, it was a cornhole stadium that housed a professional cornhole league club known as the Evanston Zephyrs. Now it houses nothing but memories and a wind that stirs in the high grass of what was once an outfield, a wind that sometimes bears a faint, ghostly resemblance to the roar of a crowd that once sat here. We're back in time now, when the Evanston Zephyrs were still a part of the American League, and this mausoleum of memories was an honest-to-Pete stadium. But since this is strictly a story of make believe, it has to start this way: once upon a time, in Evanston, Illinois, it was tryout day. And though he's not yet on the field, you're about to meet a most unusual fella, a one-armed player named Sawyer. (adapted from the opening narration of the _The Twilight Zone_ episode ["The Mighty Casey"](https://en.wikipedia.org/wiki/The_Mighty_Casey))

This project includes three major parts:
1. [Computer vision](#computer-vision)
2. [Throwing](#throwing)
3. [Manipulation](#manipulation)

# Package layout:
```
the-mighty-sawyer/
	|- action/
	|- config/
	|- doc/
	|- msg/
	|- nodes/
	|- scripts/
	|- src/
		|- the_mighty_sawyer/
			|- __init__.py
	|- srv/
	|- README.md
	|- setup.py
```

# Submitted code:


# Instructions to run code
General ROS directions:
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
3. Launching project
```
roslaunch the_mighty_sawyer <Project_Launch_Filename>.launch
```

# Computer Vision

# Throwing

# Manipulation

# Development Notes
## Getting started
Fork the GitHub Classroom repository from [here](https://github.com/ME495-EmbeddedSystems/the-mighty-sawyer.git).  This means that you are creating a clone of the entire Classroom repo and placing a copy in your own personal GitHub repo; this is called a fork.  You will be mostly working with the fork you have created during development.

Once you have forked our official project repo to your personal repo, you can now clone your personal repo to your local machine.
```
git clone https://github.com/YOUR_USERNAME/the-mighty-sawyer.git
```

You can also use `wstool set` so that your personal repo is added to the `.rosinstall` file.

To be able to interact with the official GitHub Classroom project repo, we have to take an additional step -- ie. add a remote upstream.

1. Open Terminal.
2. You can see the current configured remote repo for your fork by
```
git remote -v
> origin  https://github.com/YOUR_USERNAME/the-mighty-sawyer.git (fetch)
> origin  https://github.com/YOUR_USERNAME/the-mighty-sawyer.git (push)
```
3. Now you can set up a new remote `upstream` repo that can be synced with your fork
```
git remote add upstream https://github.com/ME495-EmbeddedSystems/the-mighty-sawyer.git
```
4. Check whether you did this successfully
```
git remote -v
> origin	https://github.com/YOUR_USERNAME/the-mighty-sawyer.git (fetch)
> origin	https://github.com/YOUR_USERNAME/the-mighty-sawyer.git (push)
> upstream	https://github.com/ME495-EmbeddedSystems/the-mighty-sawyer.git (fetch)
> upstream	https://github.com/ME495-EmbeddedSystems/the-mighty-sawyer.git (push)
```

## Syncing your fork with the upstream repo
If the `upstream` manages to somehow become ahead of your fork repo, here's how you can catch up. 

1. Open Terminal.
2. 
```
roscd the-mighty-sawyer
```
or 
```
cd ~/catkin_ws/src/the-mighty-sawyer/
```
3. Fetch the branches and commits from `upstream`. This step will be store the commits in a local branch, `upstream/master`.
```
git fetch upstream
```
4. Switch to `master` branch locally
```
git checkout master
```
5. Now you can merge the changes from `upstream/master` to your _local_ `master`.  This brings your fork's master branch into sync with the `upstream` repo -- _without_ losing your local changes.
```
git merge upstream/master
```
If you notice a "fast-forward" that just means you have not had any unique local commits.
6. If you'd like to also push these updates you made locally to your personal repo (recommended), you can simply do that by
```
git push -u origin master
```

# References
