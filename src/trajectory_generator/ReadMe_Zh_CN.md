# 轨迹规划器与控制器集成指南

本文档介绍了**轨迹规划器**节点与**控制器**节点之间的通信接口与协作细节，涵盖话题、消息类型、坐标系、发布频率及各自职责，确保闭环轨迹跟踪平稳执行。

---

## 1. 概述

- **轨迹规划器** 发布：
  - `/velocity`（`geometry_msgs/TwistStamped`）：前馈速度指令
  - `/target_pose`（`geometry_msgs/PoseStamped`）：参考位置与朝向
- **控制器** 订阅以上话题，通过闭环反馈控制修正偏差，确保车辆跟踪规划轨迹。

---

## 2. 话题与消息格式

### 2.1 `/velocity`

```yaml
std_msgs/Header header
geometry_msgs/Twist twist
```

- **twist.linear.x**（m/s）：车辆在自身坐标系（base_link）下的前进速度
- **twist.angular.z**（rad/s）：车辆绕自身 z 轴的偏航角速度
- **header.stamp**：速度指令的时间戳，应与 `/target_pose` 时间保持对齐

### 2.2 `/target_pose`

```yaml
std_msgs/Header header
geometry_msgs/Pose pose
```

- **pose.position.x, y**（m）：目标位置在世界（map）坐标系下的坐标
- **pose.orientation**（四元数）：目标朝向在世界坐标系下的表示（yaw 由 `atan2(dy,dx)` 计算）
- **header.stamp**：到达该位姿的时间戳，应与 `/velocity` 时间保持对齐

---

## 3. 坐标系约定

| 数据                 | 坐标系         | 说明                                          |
|----------------------|---------------|----------------------------------------------|
| `/target_pose.pose`  | 世界（map）   | 位置和朝向的全局参考                          |
| `/velocity.twist`    | 车辆自身（base_link） | 车辆运动的局部速度命令（v, ω）               |

- 规划器根据世界坐标下的位置变化 `atan2(dy,dx)` 计算目标 yaw。
- 控制器将世界坐标的目标位姿转换为车辆局部误差，再与速度前馈配合进行反馈控制。

---

## 4. 频率与同步

- 参数 `time_interval`：默认 0.1s → 规划器以 10Hz 发布
- 控制器 **需 ≥10Hz** 运行，确保不漏掉任何一个轨迹点

**示例：**
```cpp
ros::Rate rate(10.0);  // 与规划器 10Hz 同步
while (ros::ok()) {
  ros::spinOnce();
  // 控制逻辑...
  rate.sleep();
}
```
或使用定时器：
```xml
<node pkg="..." type="controller_node" name="controller">
  <param name="control_rate" value="10.0"/>
</node>
```
```cpp
auto period = ros::Duration(1.0 / control_rate);
ros::Timer timer = nh.createTimer(period, controlCallback);
```

---

## 5. 控制器职责

1. **订阅话题**
   - `/target_pose`：获取期望位姿 `(x*, y*, yaw*)`
   - `/velocity`：获取前馈速度 `(v*, ω*)`
2. **状态估计**
   - 从里程计或定位模块获取当前状态 `(x, y, yaw, v)`
3. **误差计算**
   - **横向误差**：当前位置到目标位置的距离
   - **航向误差**：`Δyaw = angles::shortest_angular_distance(current_yaw, target_yaw)`
4. **控制律设计**
   - 根据误差（横向、航向）计算转向命令（如 Pure Pursuit、Stanley、PID、MPC）
   - 基于前馈速度 `v*` 计算油门/制动，以满足加速度限制
5. **输出命令**
   - 将控制结果发布给执行器或底盘驱动接口

---

## 6. 关键注意事项

- **前馈 vs 反馈**：`/velocity` 提供前馈基准，`/target_pose` 与传感器反馈用于误差修正
- **坐标转换**：将世界坐标系的目标位姿转换为 base_link 下的误差；速度命令保留在 base_link
- **时间对齐**：确保 `/velocity` 与 `/target_pose` 的时间戳匹配，必要时可做缓冲或插值
- **最终停车**：规划器在末尾设置速度为 0，控制器需识别此状态并切换到空闲或下一阶段

---

## 7. Launch 示例与参数

```xml
<launch>
  <!-- 轨迹规划器 -->
  <node pkg="trajectory_generator" type="trajectory_node_v2_with_pose" name="trajectory_planner">
    <param name="time_interval" value="0.1"/>
    <param name="v_max"        value="2.0"/>
    <param name="a_max"        value="1.0"/>
    <param name="a_lat_max"    value="0.5"/>
  </node>

  <!-- 控制器 -->
  <node pkg="controller_pkg" type="controller_node" name="controller">
    <param name="control_rate" value="10.0"/>  <!-- 与规划器保持 10Hz 同步 -->
  </node>
</launch>
```

---

## 8. 故障排查

- **消息丢失？** 检查节点命名空间与话题重映射是否一致
- **时间漂移？** 避免混用系统时间与仿真时间，可考虑消息同步机制
- **跟踪误差大？** 调整控制器反馈增益，或降低规划器速度/加速度限制

---

*文档结束*

