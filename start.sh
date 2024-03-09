# Clean Build ROS Packages
./cleanbuild.sh

# Starting ROS Nodes
source install/setup.bash
ros2 run communication csv_parse &
rqt_graph