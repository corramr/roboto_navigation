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

# go to nav2_ws
WORKDIR /root/nav2_ws

# build and source application code
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select nav2_bringup --symlink-install

# build nested environments
WORKDIR /root/nav2_ws/src/ws_livox 
RUN source /opt/ros/humble/setup.bash && \
    colcon build 

# go back to nav2_ws
WORKDIR /root/nav2_ws
