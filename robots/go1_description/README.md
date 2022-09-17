# go1_description

## Usage

With real hardware. (In development Sep 17 2022)

```Shell
$ ros2 launch go1_description unitree_go1.launch.py
```

With fake hardware.

```Shell
$ ros2 launch go1_description unitree_go1.launch.py use_real_hardware:=false
```

With ignition gazebo hardware.

```Shell
$ ros2 launch go1_description unitree_go1.launch.py use_real_hardware:=false ignition_gazebo:=true
```


## Launch Arguments

| Argument | Type | Default |
| :- | :-: | :- |
| `namespace` | `string` | `''` |
| `robot_model_file` | `string` | `'unitree_go1.urdf.xacro'` |
| `robot_model_path` | `string` | `'${this_pkg}/models/urdf'` |
| `joint_state_publisher_config_file` | `string` | `'${this_pkg}/config/joint_state.yaml'` |
| `use_sim_time` | `bool` | `'false'` |
| `ros2_control_config_file` | `string` | `'${this_pkg}/config/ros2_controllers.yaml'` |
| `use_real_hardware` | `bool` | `'true'` |
| `ignition_gazebo` | `bool` | `'false'` |
| `use_rviz` | `bool` | `'false'` |
| `rviz_config_file` | `string` | `'${this_pkg}/rviz/unitree_go1.rviz'` |


## TODO

+ Add real hardware ros2_control plugin

