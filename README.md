# WORK IN PROGRESS... 




# Unity Robotics Hub - new Secenes 

This repository was forked from the [__Unity-Robotics-Hub__ ](https://github.com/Unity-Technologies/Unity-Robotics-Hub). Their repo is awesome and easy for beginners to get along. 

This Repository has some new scenes build upon the existing *Pick and Place* tutorial: 
- Place multiple objects to different target places 
- Place multiple spawned objects to different target places 
- Set the gripper to a specific location 

Following will a step by step guide how [run the scenes](#running-this-repo) of this repo, and [recreate](#starting-from-scratch) them by yourself.

---
## [Running this Repo](#running-this-repo)
### PART 1: Getting the Repo 

Start by cloning the repo, if you just want to get the scenes running clone this repo: 
```
https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
```
If you want to start from scratch clone the Unity-Robotics-Hub repo: 
```
https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
```
The new build scenes are part of the Pick-and-Place project. If you are facing problems check out the [Pick-and-Place Tutorial](tutorials/pick_and_place/README.md).

---
### Part 2: Starting the ROS Server and Unity Scene

Please follow this tutorial to set up the ROS-Server: [ROS-Tutorial](tutorials/pickplacewall/tutorials/pick_and_place/0_ros_setup.md)

Next we open the project in Unity: [Unity-Tutorial](https://github.com/lastthought15/Unity-Robotics-Hub/blob/pickplacewall/tutorials/pick_and_place/1_urdf.md)

---
### Part 3: Unity set-up 


---
### Part 4: ROS launch

To set up the Communication between Unity and we need to execute the launch file from your terminal. Make sure you run it from within the catkin workspace.
```
roslaunch niryo_moveit part_3.launch
```

---
## [Starting from Scratch](#starting-from-scratch)
### Part 1: Pick and Place Tutorial 

Start by doing all the steps described in the Pick-and-Place Tutorial from Unity-Robotics-Hub: [Pick-and-place tutorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/README.md)
