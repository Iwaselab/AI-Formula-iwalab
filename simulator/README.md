## Simulator

This package provides a simulator for AIFormula.

![simulator_readme](https://github.com/honda-hgrx-idcs/EC7D_AIformula_Control/assets/113084733/4911f1eb-dbe8-48d8-8373-0b9f52ea1f73)

#### Acquirable data：
* camera data
* odometry data

You can control the mobility of the simulator topic: `/cmd_vel`

### Running Example
To start simulator:
  ```bash
  ros2 launch simulator gazebo_simulator.launch.py
  ```

### If you want to use a gamepad in the simulator
1. Change the value of `control/speed_command/gamepad` from `"/aiformula_control/gamepad/cmd_vel"`
to `"/cmd_vel"` in `launchers/config/topic_list.yaml`
2. Start `gamepad_joy.launch.py` and `gamepad_teleop.launch.py` in launchers package.


### It takes a long time (about two and a half minutes) only when launching gazebo_simulator.launch.py `for the first time` !
- Once the world file is loaded into gazebo, it is cached under `~/.gazebo/model`
- After the second time, it does not take long because the cache can be used.
