# start from ros2 image
FROM osrf/ros:humble-desktop

# use bash
SHELL ["/bin/bash", "-c"]

# update apt repository 
RUN apt-get update
    
# Set the working directory to your workspace
WORKDIR /root/nav2_ws

# clone nav2 and install its dependencies
RUN source /opt/ros/humble/setup.bash && \
    mkdir -p src && \
    git clone https://github.com/ros-navigation/navigation2.git --branch humble ./src/navigation2 && \
    rosdep install -y \
      --from-paths ./src \
      --ignore-src
      
# Build the workspace
RUN source /opt/ros/humble/setup.bash && \ 
    colcon build \
      --symlink-install \
      --parallel-workers 1 \
      --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1
 
# clone and install slam toolbox
RUN git clone https://github.com/SteveMacenski/slam_toolbox.git --branch humble ./src/navigation2/slam_toolbox

# Build the workspace
RUN source /opt/ros/humble/setup.bash && \ 
    colcon build \
      --packages-select slam_toolbox \
      --symlink-install \
      --parallel-workers 1 \
      --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1
      
# install additional packages
RUN apt-get update && \
    apt-get install -y ros-humble-rqt-tf-tree && \
    apt-get install -y iproute2 net-tools && \
    apt install cmake

# install livox-sdk2
RUN cd /root && \
    git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd ./Livox-SDK2/ && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j1 && \
    make install


# Aggiungi usr/local/lib to dynamic linker path (so livox-sdk2 is visible to any other program)
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/usr-local-lib.conf && ldconfig

# Copia il file init.bash locale nel container (nella cartella temporanea)
COPY app/initialize/init.bash /tmp/init.bash

# Aggiungi il contenuto di init.bash in coda al .bashrc di root e rimuovi il file temporaneo
RUN cat /tmp/init.bash >> /root/.bashrc && \
    rm /tmp/init.bash

# remove cache data of APT metadata     
RUN rm -rf /var/lib/apt/lists/*
