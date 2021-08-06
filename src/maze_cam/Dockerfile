FROM ros:foxy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    python3-pip 

RUN apt install -y \
    ros-foxy-v4l2-camera 

RUN apt upgrade -y 

RUN mkdir -p /ros2_ws/src && \ 
    cd /ros2_ws && \
    colcon build --symlink-install --merge-install

RUN mkdir -p /ros2_ws/src/custom_interfaces
RUN mkdir -p /ros2_ws/src/maze_cam

COPY ./custom_interfaces /ros2_ws/src/custom_interfaces
COPY ./maze_cam /ros2_ws/src/maze_cam

RUN cd /ros2_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN cd /ros2_ws && \
    source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-args -DBUILD_TESTING=OFF --merge-install

COPY ./maze_cam/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]