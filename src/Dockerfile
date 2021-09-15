ARG ROS_DISTRO=galactic
FROM ros:$ROS_DISTRO-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    python3-pip \
    python3-opencv \
    python3-colcon-common-extensions

RUN apt update && apt install -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-rviz2

RUN apt upgrade -y 

RUN mkdir -p /ros2_ws/src

COPY ./custom_interfaces /ros2_ws/src/custom_interfaces
COPY ./maze_bringup /ros2_ws/src/maze_bringup
COPY ./maze_robot /ros2_ws/src/maze_robot

WORKDIR /ros2_ws

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-args -DBUILD_TESTING=OFF --merge-install

COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]