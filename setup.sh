#!/bin/bash

# Source Gazebo environment
source /usr/local/share/gazebo-9/setup.sh

# Export location to GAP 1.5 plugins
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/usr/local/lib/gap-1.5
# Export GAP message library
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/gap-1.5

# Export location of grasp plugins
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`/build/lib

# Other
# export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:`pwd`/models

echo "GRASP v1.0 - Successfully initialised."
