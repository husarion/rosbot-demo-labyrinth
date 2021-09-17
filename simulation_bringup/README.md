# Rosbot demo labyrinth simulation in the gazebo.
## Copy project git repository
Create a folder for your workspace and copy the git repository. 
```
mkdir -p rosbot_ws/src
cd rosbot_ws/src
git clone https://github.com/husarion/rosbot-demo-labyrinth.git
```
## Set up husarnet communication
In this project, husarnet is used to communicate between a docker container and the host machine. Husarnet allows to create a wirtual network interface on top of your operating system that allows your system to work over Internet. It is designed with ROS & ROS2 in mind and applies peer-to-peer comunication. Go to https://app.husarnet.com and create a free account or log in if you already have one. Then create a network using the “Create Network” button. Next click “Add element” and save the Join Code of your network. We will need it later.
### Set up docker connection with husarnet
Now you have to edit .env file located in `/rosbot-demo-labyrinth/src/docker_sim` folder by changing JOINCODE. 
```
HOSTNAME=maze-nav2-sim
JOINCODE=fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/xxxxxxxxxxxxxxxxxxxxxxx
```
If you don’t want to install ROS2 on your host machine you can use commands from inside a running container. It is described in the [Using ROS commands from inside container](#using-ros-commands-from-inside-container) section how to do it. If you choose to call ROS2 command from container then you can skip rest of steps in this section.
### Connect your host machine to husarnet
All you need to do to install Husarnet and connect your host to a Husarnet network is:
```
## 1. Install Husarnet
curl https://install.husarnet.com/install.sh | sudo bash

## 2. Reload systemd starting Husarnet daemon
sudo systemctl restart husarnet

## 3. Join the VPN network
sudo husarnet join ${JOINCODE} ${HOSTNAME}
```
I choose “laptop” as my hostname, if you do the same it will save you some steps.

### Set up a connection on the host machine
If you chose a different name for your host than “laptop” you also have to change the name of peer address in a cyclonedds.xml file located in `rosbot-demo-labyrinth/src/docker_sim` directory (line 13).  
```
<Peer address="laptop"/>
```
To be able to communicate with docker container you need to have rmw_cyclonedds installed on your host machine (If you are using galactic distribution for ROS2, Cyclone DDS is used as default so you just need to export  CYCLONEDDS_URI):
```
apt install ros-<ros-distro>-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/rosbot_ws/src/rosbot-demo-labyrinth/src/cyclonedds.xml
```
### Building neccessary packages
To be able to call custom interfaces made for this project you need to build some packages first. In terminal type:
```
cd ~/rosbot_ws
colcon build --packages-select custom_interfaces
source ~/rosbot_ws/install/setup.bash
```

## Launching Gazebo simulation 
To launch simulation you have to go to the `docker_sim` folder, allow a docker container to connect to your display, and then simply run the docker-compose file. Open new terminal and type:
```
cd ~/rosbot_ws/src/rosbot-demo-labyrinth/src/docker_sim
xhost local:root
sudo docker-compose up --build
```
When you start demo for the first time docker container with navigation is not yet added to a husarnet network so it won't be able to communicate with itself. You need to restart it to make everything work fine. 

To make the robot go through the labyrinth you have to call ROS service called “start” providing goal x and y position. 
For simulation, maze exit is around x=10.0, y=8.5. Go to the first terminal where you set up a connection on the host machine and type:
```
ros2 service call /start custom_interfaces/srv/Start "{x: 10.0, y: 8.5}"
```
This service will update the map and send the goal position. Then nav2 will generate a path through the labyrinth and make the robot follow it. Generated path may have little zig-zags, but a local trajectory planner should make the robot drive smoothly.

If you don’t want to install ROS2 on your host machine you can use commands from inside a running container. It is described in the [Using ROS commands from inside container](#using-ros-commands-from-inside-container) section how to do it.

## Using ROS commands from inside container
If you don’t want to install ROS2 on your host machine you can use commands from inside a running container. You can do it by using container that is running simulation:
```
sudo docker container exec -it docker_sim_maze_navigation_1 bash
```
Then inside container setup environment:
```
source /opt/ros/galactic/setup.bash
source /ros2_ws/install/setup.bash
export CYCLONEDDS_URI=file:///cyclonedds.xml
```
Now you can use ROS2 commands from inside the container. 
