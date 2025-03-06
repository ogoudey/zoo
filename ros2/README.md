# RL Zoo
## Requirements
Running this requires gazebo and ROS2.

## Running
Shell A:
This targets a world in the gz demos folder.
```
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="model.sdf" # with --headless-rendering -s
```
Add a new world with `sudo cp resources/vehicle/<name>.sdf ~/../../opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/`.

Shell B:
Start the world control bridge, for respawning the vehicle.
```
ros2 run ros_gz_bridge parameter_bridge /world/model/control@ros_gz_interfaces/srv/ControlWorld
```

Shell C:
Start the bridge for the other commands, cmd_vel and odometry, as specified in the bridge.yaml.
```
ros2 launch gz_interface gz_launcher.py bridge_name:=bridge config_file:=resources/bridge.yaml
```

Shell D:
Finally, run the control scripts.
```
colcon build
. install/setup.bash
ros2 run gz_interface controller.py
```
