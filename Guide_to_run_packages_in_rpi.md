##Going to the root
sudo -i 

## Sourcing Ros
source /home/muc/opt/ros/humble/setup.bash 

## Building package
cd /home/muc/MUC
colcon build

## Sourcing project
source install/setup.bash

##Running packages
ros2 run <package_name> <node_name>
