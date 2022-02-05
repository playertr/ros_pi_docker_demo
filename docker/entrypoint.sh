#! /bin/bash

# source ROS underlay
source /opt/ros/${ROSDISTRO}/setup.bash

# source the setup file to make our installed software available
source ${WORK_DIR}/devel/setup.bash

exec "$@"