#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#echo "This is an empty launch script. Update it to launch your application."
roslaunch augmented_reality augmented_reality.launch veh:=$VEHICLE_NAME

