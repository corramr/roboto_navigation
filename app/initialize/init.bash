source /opt/ros/humble/setup.bash
source /root/nav2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle  # Iron and older only with Gazebo Classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/initialize/tb3_gazebo_models

