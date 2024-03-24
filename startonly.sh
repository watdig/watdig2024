source install/setup.bash
ros2 run communication csv_parse &
ros2 run navigation navigator_node &
ros2 run sensor_integration front_uwb_node &
ros2 run sensor_integration gyro_node &
ros2 run localization localization_node &
rqt_graph