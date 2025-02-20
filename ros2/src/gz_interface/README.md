# ROS2-Gazebo Templates Guide

### Launching Gazebo through ROS -- Keypressng

```
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```
```
ros2 launch ros_gz_bridge ros_gz_bridge.launch.py bridge_name:=ros_gz_bridge config_file:=bridge.yaml
```
```
ros2 topic echo keyboard/keypress
```
### -- Loading a prebuilt car
```
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```
```
ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=empty file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=vehicle x:=0.0 y:=0.0 z:=0.5
```
#### "Driving"

```
ros2 launch ros_gz_bridge ros_gz_bridge.launch.py bridge_name:=ros_gz_bridge config_file:=resources/bridge.yaml
```
Then like
```
ros2 topic pub /model/vehicle/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0, y: 0.0, z: 0.0}, angular: {x: 1.0, y: 1.0, z: 0.0}}"
```
### Final Form
From WS
```
ros2 launch gz_interface gz_launcher bridge_name=bridge config_file=resources/bridge.yaml
no: ros2 launch gz_interface gz_launcher.py bridge_name:=bridge config_file:=resources/bridge.yaml

```
another terminal...
```
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```
another terminal... spawning a vehicle
```
ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=empty file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=vehicle x:=0.0 y:=0.0 z:=0.5
```

another terminal...
```
ros2 run gz_interface gz_interface
>>>>>> angular.z = z # rotate
>>>>>> linear.x = x # forward/backward
```
