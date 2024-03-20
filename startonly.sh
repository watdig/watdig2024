source install/setup.bash
ros2 run communication csv_parse &
ros2 run navigation navigator_node &
rqt_graph