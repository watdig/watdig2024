# Clean Build ROS Packages
./cleanbuild.sh

# Starting ROS Nodes
source install/setup.bash
ros2 run communication csv_parse &
ros2 run navigation navigator_node &
ros2 run communication position_subscriber &
ros2 run sensor_integration gyro_node &
ros2 run controls controls_node &
ros2 run controls action_server &
ros2 run localization localization_node &
ros2 run sensor_integration front_uwb_node &
ros2 run rosbridge_server rosbridge_websocket &
rqt_graph