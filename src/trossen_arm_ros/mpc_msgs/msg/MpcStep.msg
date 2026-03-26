# One MPC solve + send step (for rosbag). header.stamp is ROS time at publish.
std_msgs/Header header
float64 t_wall
geometry_msgs/Point ball_pos
geometry_msgs/Vector3 ball_vel
geometry_msgs/Point p_des
float64 t_strike
float64[] cur_q
float64[] cur_qd
float64 mpc_time
float64 mpc_error
# Trajectories: n_points rows x n_joints columns, row-major flatten (OpenCV/numpy C order).
uint32 n_joints
uint32 n_points
float64[] q_traj_flat
float64[] qd_traj_flat
float64[] qdd_traj_flat
