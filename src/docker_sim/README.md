# Rosbot demo labyrinth simulation in the gazebo.
## Copy project git repository
Create a folder for your workspace and copy the git repository. For example:
```
mkdir -p rosbot_ws/src
cd rosbot_ws/src
git clone https://github.com/husarion/rosbot-demo-labyrinth
```
## Set up husarnet communication
In this project, husarnet is used to communicate between a docker container and the host machine. Go to https://app.husarnet.com and create a free account or log in if you already have one. Then create a network using the “Create Network” button. Next click “Add element” and save the Join Code of your network. We will need it later.
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

### Setup connection on host machine
To be able to communicate with docker container you need to have rmw_cyclonedds installed on your host machine:
```
apt install ros-<ros-distro>-rmw-cyclonedds-cpp
```
Then go to the folder where you copied the project and on your host machine terminal:
```
cd /rosbot-demo-labyrinth/src
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/rosbot_ws/rosbot-demo-labyrinth/src/cyclonedds.xml
```
### Setup docker connection with husarnet
Now you have to edit .env file located in `/rosbot-demo-labyrinth/src/docker_sim` folder by changing JOINCODE. 
```
HOSTNAME=maze-nav2
JOINCODE=fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/xxxxxxxxxxxxxxxxxxxxxxx
```
If you choose a different name for your host than “laptop” you also have to change the name of peer address in a cyclonedds.xml file located in rosbot-demo-labyrinth/src/docker_sim directory (line 14).  
```
<Peers>
	<Peer address="maze-nav2-sim"/>
	<Peer address="laptop"/>
</Peers>
```

## Launching Gazebo simulation 
To launch simulation you have to go to the `docker_sim` folder, allow a docker container to connect to your display, and then simply run the docker-compose file.
```
cd /rosbot-demo-labyrinth/src/docker_sim
xhost local:root
docker-compose up --build
```
When we start demo for the first time docker container with navigation is not yet added to a husarnet network so it won't be able to communicate with itself. We need to restart it to make everything work fine. 

To make the robot go through the labyrinth you have to call ROS service called “start” providing goal x and y position. 
For simulation, maze exit is around x=10.0, y=8.0. 
```
ros2 service call /start custom_interfaces/srv/Start "{x: 10.0, y: 8.0}"
```
This service will update the occupancy map and send the goal position. Then nav2 will generate a path through the labyrinth and make the robot follow it.
