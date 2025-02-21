# System Installation
 - `My System` : Ubuntu 22.04 with 16 GB RAM, Nvidia GTX 1650Ti with 4GB GPU Memory
 - `ROS2 Installation`: [Installation Instruction Link](https://docs.ros.org/en/jazzy/Installation.html)
 - `NVIDIA DRIVER Installation` : [Installation Instruction Link](https://ubuntu.com/server/docs/nvidia-drivers-installation)
 - `NVIDIA Docker2 Installation`: [Installation Instruction Link](https://www.server-world.info/en/note?os=Ubuntu_22.04&p=nvidia&f=2)
 - `Docker Installation`: [Installation Instruction Link](https://docs.docker.com/engine/install/)
# Docker
 - Download this repository and go to the Docker folder. Ensure that Docker and Nvidia Docker2 is installed. Open a new termial inside the Docker folder or give the path in the following instruction
 
```
sudo docker build -t <name_of_dockerfile>:<tag of dockerfile> <path>
```
 If you are inside the Docker folder
 
```
sudo docker build -t osrf/ros:jazzy-harmonic .
```
 else give the full path
```
sudo docker build -t osrf/ros:jazzy-harmonic /home/Desktop/Docker
```
 
 Open a new terminal and mount this folder into the ROS2 workspace as given in the following command

```
sudo docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="/home/soumic/Desktop/acceleration_robotics/src:/home/ros/docker_navigation/src" \
    --net=host \
    --privileged \
    --gpus=all \
    osrf/ros:jazzy-harmonic:latest \
    bash   
```
 - `--volume="/home/soumic/Desktop/acceleration_robotics/src:/home/ros/docker_navigation/src"` Please use your folder path of this dictory using `pwd` command a place in the left side of the colon and on the right side the path to your ROS2 workspace inside the Docker.
 - To use multiple instances of the same docker instance use the command
```
sudo docker ps
```
and then use the docker id of the instance in the following command
```
sudo docker exec -it <docker-instance-id> bash
```
This is required when one wants to run multiple ros2 commands in the same docker instance.

# Turning on the X11 server (in the system, not inside docker instance)
One needs to disable the access control on the x11 server for the display. Also check the `echo $DISPLAY` for the display number.
```
xhost +
```

# Clone Slam_toolbox and Nav2 in your ROS2 workspace (colcon_ws/src or ROS2_ws/src)
Please do not forget to add Jazzy Branch of the repositories 
```
git clone https://github.com/SteveMacenski/slam_toolbox.git -b jazzy
```
```
git clone https://github.com/ros-navigation/navigation2.git -b jazzy
```

# ROS Dependencies Installation

```
rosdep update
```
```
rosdep install -q -y -r --from-paths src --ignore-src
```
if you hare using some ros version which has reached end-of-life (eol) add --eol $ROS_DISTRO as a parameter to the above command.

```
colcon build --symlink-install
```
If your system hangs, check the number of threads in your system using `nproc` command. Then reduce the parallel workers as follows
```
colcon build --symlink-install --parallel-workers 3
```
Always source the following command whenever running a ROS2 command as
```
source /opt/ros/jazzy/setup.bash
```
```
source install/setup.bash
```
# DDS version
By default ROS2 uses Fast DDS, but if cyclone DDS is installed in the docker. If you want to switch to cyclone dds use
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

# Gazebo World with Robot

```
ros2 launch diff_drive_robot robot.launch.py
```
One can download as much world as needed in the "world" folder of the "diff_drive_robot" folder and download different differential drive robot models from [fuel model resources](https://app.gazebosim.org/fuel/models) and [insert models into the gazebo world](https://gazebosim.org/docs/latest/fuel_insert/) or write a ROS2 node/ launch to spawn the Robot in the World. 

Note that Gazebo keeps running sometimes in the background after you finish working and do some modification and wants to run gazebo again please check
```
ps aux | grep gz
```
If still running kill the process
```
kill -9 <gz-instance-id>
```
One can find the `gz-instance-id` from the `ps` command above.

# Teleoperation
As the Gazebo-ROS2 bridge is already install, one can either use the Gazebo 'Teleop' option from the Gazebo window or use ROS2 teleoperation node to move the robot
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# Mapping
modify the `mapper_params_online_async.yaml` file in the slam_toolbox to mapping and then run the following command
```
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/ar_task_amr/diff_drive_robot/config/mapper_params_online_async.yaml use_sim_time:=true
```
Use Teleoperation node or Gazebo Teleoperation to move the robot in the world. Do not move the robot fast else point clouds might not be properly captured in the map - erroneous map results.

Please select the Slam option from 'add panel' of Rviz to save the map using Graphical Interface. Else from command line

```
ros2 run nav2_map_server map_server_cli -f my_map
```

# Setting RViz
 - Add a 'map' from 'add' button
 - set 'topic' to 'map'
 - choose Transient Local'
 - set 'map' frame in the global options to visualize the map
 
The map_server of nav2 must be running or run the following command. One can store the saved map anywhere in the repository but the correct path should be given. 

```
ros2 launch diff_drive_robot map_server.launch.py map:=/home/ros/docker_navigation/my_map_save.yaml
```

# Localization

One can check how localization performs either with the default localization of the slam_toolbox or Adaptive Monte Carlo Localization (AMCL) in nav2.

 - `slam_toolbox` : change the mapping configuration to localization in the `mapper_params_online_async.yaml` file and run again 
```
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/ar_task_amr/diff_drive_robot/config/mapper_params_online_async.yaml use_sim_time:=true
```
 - `AMCL nav2` :  check the defalut configuration to localization in the `nav2_params.yaml` file and modify the AMCL parameters accoringly depending upon vehicle dynamics and odometry, Lidar sensor specification and limits, Particle Filter tuning parameters, etc and run
```
ros2 launch nav2_bringup localization_launch.py map:=/home/ros/docker_navigation/src/ar_task_amr/my_map_save.yaml use_sim_time:=true
```
# Setting RViz
 - Add a 'map' from 'add' button
 - set 'topic' to 'global_planner/costmap'
 - choose Transient Local'
 - in the map select costmap
 - set 'map' frame in the global options to visualize the map
 
The AMCL will look for an initial guess of the initial pose. Choose 'set initial pose' from the panel and select '2D pose estimate'. select an initial position around the robot appears white in the given map. Once the initial pose is selected the robot reappears in the guessed pose in the map. Please select an initial pose within the error ellipse provided in the AMCL parameters (alpha1-4, set alpha5 to 0 if one is not using omni-directional robot) in the `nav2_params.yaml` file.

Note that without fixing the Localization parameters there is no point of proceding further. All the rest will be erroneous.

# Runing single and multiple waypoints/ goal point(s)
 - run the following command for single waypoint navigation in another terminal/ docker instance
```
ros2 run diff_drive_robot waypoint_follower_single
```
 - run the following command for multiple waypoint navigation in another terminal/ docker instance
```
ros2 run diff_drive_robot waypoint_follower_mult
```

# Global and Local Planner Tuning

There are two parameters - `planner_server` for Global planner and `controller_server` in the `nav2_params.yaml` file which need to be tuned for improving Global and Local planning strtegies.
 - `planner_server` (Global Planner - Parameters): references -> [main](https://docs.nav2.org/configuration/packages/configuring-planner-server.html), [NavFn Planner](https://docs.nav2.org/configuration/packages/configuring-navfn.html#parameters), [smac Planner](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html), [Theta Star Planner](https://docs.nav2.org/configuration/packages/configuring-thetastar.html)
 - `controller_server` (Local Planner - Parameters) : references -> [main](https://docs.nav2.org/configuration/packages/configuring-controller-server.html), [DWB Controller](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html), [MPPI Controller](https://docs.nav2.org/configuration/packages/configuring-mppic.html), [RPP Controller](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html#regulated-pure-pursuit-parameters), [RS Controller](https://docs.nav2.org/configuration/packages/configuring-rotation-shim-controller.html), [Graceful Controller](https://docs.nav2.org/configuration/packages/configuring-graceful-motion-controller.html)
 - Notes: I have selected `smac planner` as a Global panner and `MPPI Controller` as a Local Planner.
 
# Introduce Dynamic Obtacles in Gazebo World

One can follow [this](https://gazebosim.org/docs/latest/actors/) to add a moving person called `Actor` in the Gazebo World.
 
# Replanning Strtegies
 - To address replanning in presence of Dynamic Obstacles one can modify `nav2_bt_navigator/behavior_trees/*.xml`. The parameters `<RecoveryNode number_of_retries="6" name="NavigateRecovery">` can be increased (10 may be). Also recovery behavior can be introduced in the `spin_server`.
 





















































