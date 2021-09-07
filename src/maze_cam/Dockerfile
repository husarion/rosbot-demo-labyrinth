ARG ROS_DISTRO=galactic
FROM ros:$ROS_DISTRO-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    python3-pip 

RUN apt update && apt install -y \
    ros-$ROS_DISTRO-v4l2-camera 

RUN apt upgrade -y 

# RUN apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp
RUN sudo rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src

COPY ./custom_interfaces /ros2_ws/src/custom_interfaces
COPY ./maze_cam /ros2_ws/src/maze_cam

WORKDIR /ros2_ws

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-args -DBUILD_TESTING=OFF --merge-install

COPY ./maze_cam/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]