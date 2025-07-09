#Copyright 2025 by Jia Cao
# Trajectory Planner & Controller Integration Guide

This document describes the communication interface and coordination between the **Trajectory Planner** node and the **Controller** node. It outlines the topics, message types, coordinate frames, frequencies, and responsibilities to ensure smooth closed-loop trajectory tracking.

---

## 1. Overview

- **Trajectory Planner** publishes:
  - **/velocity** (`geometry_msgs/TwistStamped`): feedforward speed commands
  - **/target\_pose** (`geometry_msgs/PoseStamped`): reference positions & orientations
- **Controller** subscribes to these topics to perform closed-loop feedback control, correcting deviations and ensuring the vehicle follows the planned trajectory.

---

## 2. Topics & Message Formats

### 2.1 /velocity

```yaml
Header header
geometry_msgs/Twist twist
```

- **linear.x** (m/s): forward speed in **base\_link** frame
- **angular.z** (rad/s): yaw rate (rotation about vehicle’s local z axis) in **base\_link** frame
- **header.stamp**: time of the command (should align with /target\_pose stamp)

### 2.2 /target\_pose

```yaml
Header header
geometry_msgs/Pose pose
```

- **pose.position.x, y** (m): goal position in **world** (map) frame
- **pose.orientation** (quaternion): goal orientation in **world** frame (yaw computed via `atan2(dy, dx)`)
- **header.stamp**: time the pose should be reached (aligns with /velocity stamp)

---

## 3. Coordinate Frame Conventions

| Data               | Coordinate Frame | Notes                                 |
| ------------------ | ---------------- | ------------------------------------- |
| /target\_pose.pose | world (map)      | Global reference for position and yaw |
| /velocity.twist    | base\_link       | Local vehicle motion commands (v, ω)  |

- The planner computes yaw from world-frame positions using `atan2(dy, dx)`.
- The controller converts world-frame target\_pose to local error in base\_link for feedback.

---

## 4. Frequency & Synchronization

- **time\_interval** (parameter): default **0.1s** → Planner publishes at **10 Hz**.
- **Controller** must run at **≥10 Hz** to avoid missing commands.

**Examples:**

```cpp
ros::Rate rate(10.0);  // match planner’s 10 Hz
while (ros::ok()) {
  ros::spinOnce();
  // control logic...
  rate.sleep();
}
```

or using a timer:

```xml
<node pkg="..." type="controller_node" name="controller">
  <param name="control_rate" value="10.0"/>
</node>
```

and in code:

```cpp
auto period = ros::Duration(1.0 / control_rate);
ros::Timer timer = nh.createTimer(period, controlCallback);
```

---

## 5. Controller Responsibilities

1. **Subscription**
   - `/target_pose` → get desired pose (x\*, y\*, yaw\*) at time t
   - `/velocity` → get feedforward commands (v\*, ω\*) at time t
2. **State Estimation**
   - Obtain current state (`x, y, yaw, v`) from odometry or localization in world frame
3. **Error Computation**
   - **Lateral error**: distance from current position to target position
   - **Heading error**: `Δyaw = angles::shortest_angular_distance(current_yaw, target_yaw)`
4. **Control Law**
   - Compute steering command (e.g., Pure Pursuit, Stanley, PID, MPC) based on errors
   - Compute throttle/brake command to track feedforward `v*` and maintain acceleration limits
5. **Command Output**
   - Send low‑level drive commands to actuators/drivechain

---

## 6. Key Considerations

- **Feedforward vs Feedback**: Use `/velocity` as **feedforward** base, and correct deviations via feedback from `/target_pose` and vehicle sensors.
- **Coordinate transforms**: Convert world-frame `target_pose` to base\_link errors; twist commands remain in base\_link.
- **Time alignment**: Match timestamps between `/target_pose` and `/velocity`. Buffer or interpolate if minor mismatches occur.
- **Final stop**: Planner sets final speed to zero (`v[N-1] = 0, ω[N-1] = 0`). Controller must detect this and transition to idle or next behavior.

---

## 7. Launch & Parameters

Example launch snippet:

```xml
<launch>
  <!-- Trajectory Planner -->
  <node pkg="trajectory_generator" type="trajectory_node_v2_with_pose" name="trajectory_planner">
    <param name="time_interval" value="0.1"/>
    <param name="v_max"        value="2.0"/>
    <param name="a_max"        value="1.0"/>
    <param name="a_lat_max"    value="0.5"/>
  </node>

  <!-- Controller -->
  <node pkg="controller_pkg" type="controller_node" name="controller">
    <param name="control_rate" value="10.0"/>  <!-- Match planner 10Hz -->
  </node>
</launch>
```

---

## 8. Contact & Troubleshooting

- **Missing messages?** Ensure both nodes share the same namespace or topic remappings.
- **Time drift?** Use `ros::Time::now()` consistently; consider message filters or approximate time synchronization if needed.
- **Excessive deviation?** Increase controller feedback gains or reduce planner speed/acceleration limits.

---

*End of README.md*

