# pull from base image
FROM corra09/nav2_docker

# set shell to bash
SHELL ["/bin/bash", "-c"]

# install additional packages
RUN sudo apt update
RUN sudo apt install -y ros-humble-urdf-tutorial

# copy application code
COPY ./app/nav2_ws/src /root/nav2_ws/src
COPY ./app/initialize /root/initialize

# set working directory
WORKDIR /root/nav2_ws

# build and source application code
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select nav2_bringup --symlink-install

