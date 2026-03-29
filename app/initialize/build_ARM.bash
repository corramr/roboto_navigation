# refresh apt repository and install missing dependencies
apt-get update
apt-get install -y ros-humble-pcl-ros ros-humble-pcl-conversions

# go to dep workspace
cd /root/deps_ws

### TEMP: must be moved into the dockerfile
# Clone perception_pcl (which contains pcl_ros and pcl_conversions) directly into src
git clone https://github.com/ros-perception/perception_pcl.git --branch humble src/perception_pcl
git clone https://github.com/ros-geographic-info/geographic_info.git --branch ros2 src/geographic_info
git clone https://github.com/cra-ros-pkg/robot_localization.git --branch humble-devel src/robot_localization

colcon build \
  --parallel-workers 1 \
  --packages-select geographic_msgs pcl_conversions pcl_ros robot_localization\
  --symlink-install \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DAMENT_CMAKE_ENABLE_TESTING=OFF \
    -DCMAKE_CXX_FLAGS="-Wl,--no-keep-memory"
### /TEMP 

# go to nav2 workspace
cd /root/nav2_ws

# source ros2 and deps workspace environment
source /opt/ros/humble/install/setup.bash
source /opt/ros/humble/setup.bash
source /root/deps_ws/install/setup.bash

# build livox ros2 driver
src/livox_ros_driver2/build.sh humble

# install ros2 dep
rosdep install --from-paths src --ignore-src -y --skip-keys "gazebo_ros_pkgs"

# build other ros2 packages
colcon build --parallel-workers 1 --packages-select livox_converter --symlink-install
colcon build --parallel-workers 1 --packages-select pointcloud_to_laserscan --symlink-install
colcon build --parallel-workers 1 --packages-select fast_lio --symlink-install
colcon build --parallel-workers 1 --packages-select sensor_launcher --symlink-install
colcon build --parallel-workers 1 --packages-select cmd_vel_serial --symlink-install
colcon build --parallel-workers 1 --packages-select sentry_navigation --symlink-install
colcon build \
    --symlink-install \
    --parallel-workers 1 \
    --packages-ignore nav2_system_tests \
    --base-paths src/navigation2 \
    --cmake-args \
    -DCMAKE_BUILD_PARALLEL_LEVEL=1 \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Wl,--no-keep-memory" \
    -DBUILD_TESTING=OFF \
    -DAMENT_CMAKE_ENABLE_TESTING=OFF \
    -Dompl_DIR=/opt/ros/humble/share/ompl/cmake

# source nav2_ws environment
source install/setup.bash
