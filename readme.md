trajectory_generator节点已经添加，如果pull了我的branch，我已经把car_positions.txt调整到path_planner 包下面。
这个positions由于我的电脑比较卡，里面的点的坐标可能不是特别完美，我们后面可以再修改一下，毕竟提交给教授的版本肯定不能让别人自己跑一遍。
调试的时候，运行  "rosrun trajectory_generator trajectory_generator_node _time_interval:=0.1"，其中  _time_interval:=0.1  是参数，设置时间间隔。
再直接rosrun path_planner short_term_planner_node，rosrun path_planner path_planner_node这2个节点。
再rostopic echo /trajectory/points就能看见发布到topic: /trajectory，message类型：trajectory_msgs/MultiDOFJointTrajectory 的信息。
结构为：
controller订阅这个topic就好。

消息的结构：
header.frame_id: 
joint_names[0]: 一般为 "base_link"（可忽略）
points[]: 每一个轨迹点
transforms[0].translation.{x,y,z}：位置
transforms[0].rotation.{x,y,z,w}：朝向（四元数）
velocities[0].linear.{x,y,z}：线速度
accelerations[0].linear.{x,y,z}：线加速度（现在和小孔说不用，但是由于不同时刻速度的改变，这一项一般都不为0，后面再讨论controller需要什么信息）
time_from_start：从轨迹起始点的累积时间戳（控制器用它来插值）

试的时候直接wasd控制小车跟着绿线尽量靠近下一个点，触发下一个goal的发布。dummy controller不能跟随控制