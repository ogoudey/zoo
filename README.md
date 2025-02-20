## Zoo
Some steps to put a simulate a robot

1. Take a URDF
2. Turn into SDF with `gz sdf -p _.urdf > _.sdf
3. Make sure relative URIs are reset (make a new folder for this robot)
4. Make a ros workspace and build the package `gz_interface`
4.5. Make a resources/ directory and put the robot folder in it.
5. `ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf`
6. `ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=empty file:=resources/<robot_name>/model.sdf entity_name:=vehicle x:=0.0 y:=0.0 z:=0.5`
7. Play, and:
OPTIONAL:
8. Start a gz_ros_bridge
9. Run the gz_interface and start sending commands.

Credit for the Atlas of course goes to Boston Dynamics, found [here](https://github.com/openai/roboschool/tree/1.0.49/roboschool/models_robot).

Welcome to the Zoo!
