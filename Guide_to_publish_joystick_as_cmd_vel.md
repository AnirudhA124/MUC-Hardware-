## General 
1.Conenct the rpi and the laptop to the same network.


2.Connect the joystick to the laptop.


## Running the joy node to get inputs


```
ros2 run joy joy_node 
```

Note: To check if joystick is publishing values
```
ros2 topic echo /joy
```

## Converting joy to twist messages.

```
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'  
```

NOTE: 1.Make sure you are pressing the RB on the controller for the values to be registered and updated.

```
ros2 topic echo /cmd_vel
```

## Running the nodes((in the specific packages directory in the laptop))
```
source install/setup.bash 
ros2 run <package_name> <node_name>
```

