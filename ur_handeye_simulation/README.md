# ur_handeye_simulation

## Worlds

```bash
/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf
```


## Control

```bash
ros2 topic pub /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {stamp: {sec: 0, nanosec: 0}},
  joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
  points: [
    {
      positions: [0.0, -1.57, 1.57, 0.0, 0.0, 0.0],
      time_from_start: {sec: 3, nanosec: 0}
    }
  ]
}
```

Note that /forward_position_controller ignore dynamics (used for testing control interfaces, not physical realism)
```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]"
```


```bash
ros2 launch ur_handeye_simulation sim_moveit.launch.py handeye_setup:=eye_to_hand
```