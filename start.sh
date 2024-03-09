# Remove all built directories
echo "Removing ROS Directories."
rm -r ./build &
rm -r ./log &
rm -r ./install &

# Wait for removes to complete
wait
echo "ROS Directories removed successfully."

# Rebuilding ROS Directories and Sourcing setup
echo "Building ROS Directories"
colcon build
echo "Colcon Build Succesful. ROS Directories Recreated"
source install/setup.bash

# Starting ROS Nodes
ros2 run communication csv_parse &
rqt_graph