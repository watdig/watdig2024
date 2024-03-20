# Source the setup file
source install/setup.bash

# Define the list of node names
NODES=("csv_parse" "navigator_node" "controls_node" "action_server" "localization_node" "back_uwb_node" "front_uwb_node" "rqt_graph")

# Check if there are any nodes running
if [ ${#NODES[@]} -gt 0 ]; then
    # Iterate through each node, get its PID, and kill it
    for NODE in "${NODES[@]}"; do
        # Get the PID of the node
        NODE_PID=$(ps -e | grep $NODE | awk '{print $1}')
        
        if [ -n "$NODE_PID" ]; then
            # Kill the node using its PID
            kill $NODE_PID
            echo "Node $NODE (PID: $NODE_PID) killed."
        else
            echo "Failed to retrieve PID for node $NODE."
        fi
    done
else
    echo "No nodes running."
fi
