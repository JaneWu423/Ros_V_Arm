# CS3891 Final Project: RRT Algorithm Reproduction

## Description

This project aims to configure a dual arm setup and instruct the arms to form a V shape using the RRT planner.

## Run the Code 

To launch the project:

1. Clone down / download the repo.
2. Make the folder a catkin workspace (catkin init, catkin build)
3. Make sure to `source devel/setup.bash` at the root level of workspace and `roscd assignment`
4. type `roslaunch assignment planner.launch` to launch the GUI
5. After entering moveit.rviz GUI, for Context select CS3891Planner
6. For Planning, select plan group to be `both_arms`
7. Click plan, and arms will form a V shape, you can also add obstacle but make sure not to overlap with the final V pose.