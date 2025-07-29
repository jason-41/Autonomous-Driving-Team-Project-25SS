# Trajectory Generator README
The FINAL Docunmentation and general readme.md file may be the only explanatory documents that are the most up-to-date.

## 1. Node Function Overview

This node is part of the autonomous driving pipeline. It belongs to the `trajectory_generator` package (the sole node in that package) and bridges the global path planner and the vehicle’s controller. It converts a planned geometric path into a time-parameterized trajectory with velocity and orientation commands.

- **Package**: `trajectory_generator`  
- **Node name**: `trajectory_generator_node`  
- **Subscriptions**:  
  - `/planned_path` (`nav_msgs/Path`): discrete waypoints from the upstream planner  
  - `/Unity_ROS_message_Rx/OurCar/CoM/pose` (`geometry_msgs/PoseStamped`): current vehicle pose for nearest-point matching and closed-loop progression  
- **Timer callback**: runs at `1 / time_interval` Hz (default 10 Hz)  
- **Publications**:  
  - `/velocity` (`geometry_msgs/TwistStamped`): feed-forward linear and angular velocity in the vehicle’s `base_link` frame  
  - `/target_pose` (`geometry_msgs/PoseStamped`): lookahead reference pose `(x, y, yaw)` in the world/map frame  

---

## 2. Trajectory Generation Logic

1. **Path Preprocessing**  
   - Extracts `x[i], y[i]` from each waypoint.  
   - Computes heading `yaw[i] = atan2(y[i]–y[i–1], x[i]–x[i–1])`.  
   - Computes segment lengths `ds[i] = √((x[i+1]–x[i])² + (y[i+1]–y[i])²)`.

2. **Curvature & Speed Limit Calculation**  
   - Uses the three-point method to compute curvature `k[i]` at each waypoint:  
     \[
       k[i] = \frac{4 \times \text{area of triangle}(i-1,i,i+1)}{a \, b \, c}
     \]  
     where \(a, b, c\) are the lengths of the three sides.  
   - Computes lateral-acceleration-limited speed:  
     \[
       v_{\text{limit}}[i] = \min\!\Bigl(v_{\max},\;\sqrt{\tfrac{a_{\text{lat\_max}}}{|k[i]|}}\Bigr)
     \]

3. **Velocity Profiling**  
   - Forward pass:  
     \[
       v[i] = \min\!\bigl(v_{\text{limit}}[i],\;\sqrt{v[i-1]^2 + 2\,a_{\max}\,ds[i-1]}\bigr)
     \]  
   - Backward pass:  
     \[
       v[i] = \min\!\bigl(v[i],\;\sqrt{v[i+1]^2 + 2\,a_{\max}\,ds[i]}\bigr)
     \]  
   - Ensures a smooth, acceleration-constrained speed profile.

4. **Timestamp Assignment**  
   - \(t[0] = 0\)  
   - For \(i\ge1\):  
     \[
       t[i] = t[i-1] + \begin{cases}
         ds[i-1]/v[i], & v[i] > 10^{-3} \\
         \text{time\_interval}, & \text{otherwise}
       \end{cases}
     \]

5. **Timer-based Publishing Loop**  
   On each timer tick:  
   - **Nearest-point matching**: find the smallest index `i0 ≥ previous_i0` whose `(x[i0],y[i0])` is closest to the current pose.  
   - **Lookahead**: advance from `i0` by arc-length ≥ `lookahead_dist` to select index `j`.  
   - **Publish**  
     - `/velocity`:  
       ```  
       linear.x  = interpolated v[j]  
       angular.z = v[j] * k[j]  
       ```  
     - `/target_pose`:  
       ```  
       position    = (x[j], y[j])  
       orientation = quaternion from yaw[j]  
       ```

6. **Endpoint Handling**  
   - When distance to final waypoint ≤ `hold_dist`, hold the last computed velocity constant.  
   - After traveling an additional `stop_after_dist`, publish zero velocity and stop the timer.

---

## 3. Technical Highlights

- **Nearest-point matching + lookahead** for robust, forward-only indexing  
- **Three-point curvature estimation** for realistic cornering speeds  
- **Dual-pass acceleration-limited profiling** for smooth velocity transitions  
- **Time-parameterized trajectory** for precise, time-based control  
- **Endpoint “hold → coast → stop”** logic to prevent overshoot or oscillation

---

## 4. Key Topics & Messages

| Topic                                   | Type                         | Description                                                  |
|-----------------------------------------|------------------------------|--------------------------------------------------------------|
| `/planned_path`                         | `nav_msgs/Path`              | Discrete world/map waypoints from upstream planner           |
| `/Unity_ROS_message_Rx/OurCar/CoM/pose` | `geometry_msgs/PoseStamped`  | Current vehicle pose for nearest-point matching              |
| `/velocity`                             | `geometry_msgs/TwistStamped` | Feed-forward: `linear.x` (m/s), `angular.z` (rad/s) in `base_link` |
| `/target_pose`                          | `geometry_msgs/PoseStamped`  | Lookahead reference pose: position + yaw in world/map frame  |

---

## 5. Core Parameters (private `~`)

| Name                | Default | Unit | Description                                                     |
|---------------------|---------|------|-----------------------------------------------------------------|
| `wheel_base`        | 2.63    | m    | Vehicle wheelbase                                              |
| `v_max`             | 20.0    | m/s  | Maximum linear speed                                           |
| `a_max`             | 400.0   | m/s² | Maximum forward acceleration                                   |
| `a_lat_max`         | 400.0   | m/s² | Maximum lateral acceleration                                   |
| `time_interval`     | 0.1     | s    | Timer interval (publishing rate = 1/time_interval Hz)          |
| `lookahead_dist`    | 1.0     | m    | Arc-length ahead of nearest point to pick control target       |
| `hold_dist`         | 1.0     | m    | Distance to final waypoint at which to hold last velocity      |
| `stop_after_dist`   | 3.0     | m    | Additional distance beyond `hold_dist` before issuing final stop |

---

*End of README*  
