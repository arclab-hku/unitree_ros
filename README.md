# QRC Version 

## Usage


1. Clone the simulation map package in the `src` folder of your `workspace` .
```bash
git clone https://github.com/teamgrit-lab/ICRA2024_Quadruped_Robot_Challenges.git
```

2. Clone the rl controller package in the `src` folder and chekcout to `dev/QRC` (you might ask administrator for accessment).
```bash
git clone https://github.com/arclab-hku/Arclab_RL_Controller.git -b dev/QRC
```

3. Clone this repository in the same palce and chekcout to `dev/QRC`.
```bash
git clone https://github.com/arclab-hku/unitree_ros.git -b dev/QRC
```

4. Build and in your `workspace`, only tested in Aliengo now.
```bash
catkin build aliengo_description aliengo_rl_controller unitree_gazebo unitree_legged_msgs unitree_legged_control unitree_controller  gazebo_ros_p3d_tf extend_robot_state_publisher ICRA2024_Quadruped_Competition
```
5. Launch it!
```
roslaunch unitree_gazebo robot_map.launch map:=flat rname:=aliengo
roslaunch aliengo_rl_controller ESTController.launch
rosrun joy joy_node
```