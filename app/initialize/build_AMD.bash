# refresh apt repository
apt-get update

# go to workspace
cd /root/nav2_ws

# build livox ros2 driver
src/livox_ros_driver2/build.sh humble

# install ros2 dep
rosdep install --from-paths src --ignore-src -y  --skip-keys pyserial

# build other ros2 packages
colcon build --parallel-workers 1 --packages-select livox_converter --symlink-install
colcon build --parallel-workers 1 --packages-select pointcloud_to_laserscan --symlink-install
colcon build --parallel-workers 1 --packages-select fast_lio --symlink-install
colcon build --parallel-workers 1 --packages-select sensor_launcher --symlink-install
colcon build --parallel-workers 1 --packages-select sentry_navigation --symlink-install
colcon build --parallel-workers 1 --packages-select cmd_vel_serial --symlink-install
colcon build --parallel-workers 1 --base-paths src/navigation2 --symlink-install

# source environment
source install/setup.bash