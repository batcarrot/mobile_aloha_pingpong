#!/usr/bin/env python3
import os
import sys
from pathlib import Path

_script_dir = Path(__file__).resolve().parent
if str(_script_dir) not in sys.path:
    sys.path.insert(0, str(_script_dir))

# When run via symlink (e.g. ros2 run), __file__ is the source path (scripts/);
# the mpc_ros package lives at package_root/mpc_ros/, so add package_root to path.
_script_dir = Path(__file__).resolve().parent
_pkg_root = _script_dir.parent if _script_dir.name == "scripts" else _script_dir
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3
from cv_bridge import CvBridge
import time
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from control_msgs.msg import JointTrajectoryControllerState

import casadi

from physics_utils import BallModel
from mpc_ros import (
    AlohaArmCasadi,
)

from rclpy.action import ActionClient
from rclpy.constants import S_TO_NS
from trajectory_msgs.msg import JointTrajectoryPoint
from mpc_ros.nodes import BallPredictionNode
from physics_utils.common import *

from mpc_msgs.msg import MpcStep

_NAN = float("nan")


class MpcLogNode(Node):
    """Publishes MpcStep on /mpc_log/mpc_step for rosbag."""

    def __init__(self):
        super().__init__("mpc_log")
        self._pub = self.create_publisher(MpcStep, "mpc_step", 10)

    def publish_step(self, msg: MpcStep) -> None:
        self._pub.publish(msg)


def build_mpc_step_msg(
    clock_node: Node,
    t_wall: float,
    ball_pos: np.ndarray | None,
    ball_vel: np.ndarray | None,
    p_des: np.ndarray | None,
    t_strike: float | None,
    q_sol: np.ndarray | None,
    qd_sol: np.ndarray | None,
    qdd_sol: np.ndarray | None,
    cur_q: np.ndarray | None,
    cur_qd: np.ndarray | None,
    mpc_time: float | None,
    mpc_error: float | None,
) -> MpcStep:
    msg = MpcStep()
    msg.header.stamp = clock_node.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    msg.t_wall = float(t_wall)

    def _point3(arr: np.ndarray | None):
        if arr is None or len(arr) < 3:
            return (_NAN, _NAN, _NAN)
        return (float(arr[0]), float(arr[1]), float(arr[2]))

    bx, by, bz = _point3(ball_pos)
    msg.ball_pos.x, msg.ball_pos.y, msg.ball_pos.z = bx, by, bz
    vx, vy, vz = _point3(ball_vel)
    msg.ball_vel.x, msg.ball_vel.y, msg.ball_vel.z = vx, vy, vz
    px, py, pz = _point3(p_des)
    msg.p_des.x, msg.p_des.y, msg.p_des.z = px, py, pz
    msg.t_strike = float(t_strike) if t_strike is not None else _NAN

    msg.cur_q = cur_q.astype(np.float64).tolist() if cur_q is not None else []
    msg.cur_qd = cur_qd.astype(np.float64).tolist() if cur_qd is not None else []

    msg.mpc_time = float(mpc_time) if mpc_time is not None else _NAN
    msg.mpc_error = float(mpc_error) if mpc_error is not None else _NAN

    if q_sol is not None and q_sol.size > 0:
        msg.n_joints = int(q_sol.shape[1])
        msg.n_points = int(q_sol.shape[0])
        msg.q_traj_flat = q_sol.astype(np.float64).flatten().tolist()
        if qd_sol is not None and qd_sol.size > 0:
            msg.qd_traj_flat = qd_sol.astype(np.float64).flatten().tolist()
        if qdd_sol is not None and qdd_sol.size > 0:
            msg.qdd_traj_flat = qdd_sol.astype(np.float64).flatten().tolist()
    else:
        msg.n_joints = 0
        msg.n_points = 0
    return msg

class ArmStateNode(Node):
    def __init__(self):
        super().__init__('arm_state_node')
        self.q = None
        self.qd = None

        self.create_subscription(JointTrajectoryControllerState, '/arm_controller/controller_state', self._cb_ctrl_state, 1)
    
    def _cb_ctrl_state(self, msg: JointTrajectoryControllerState):
        self.q = np.array(msg.feedback.positions)
        self.qd = np.array(msg.feedback.velocities)

class ArmNode(Node):
    """Demo node for sending trajectory goals to the Trossen Arm's arm_controller."""

    def __init__(
        self,
        namespace: str = '',
        action_name: str = '/arm_controller/follow_joint_trajectory',
    ):
        """
        Initialize the ArmDemo node and connect to the arm_controller action server.

        :param namespace: Optional namespace to prepend to the action name and joint names.
        :param action_name: Optional name of the FollowJointTrajectory action server.
        """
        super().__init__('arm_demo')
        self.joint_names = [
            'joint_0',
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
        ]

        if namespace:
            action_name = f'{namespace}/{action_name}'
            for i in range(len(self.joint_names)):
                self.joint_names[i] = f'{namespace}/{self.joint_names[i]}'

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            action_name=action_name,
        )
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for '{self._action_client._action_name}' action server..."
            )
        self._is_running = False
        self.get_logger().info(f'ArmDemo initialized with action server: {action_name}')

    def _feedback_callback(self, feedback_msg: FollowJointTrajectory.Impl.FeedbackMessage):
        """
        Log the current joint positions from the action server's feedback.

        :param feedback_msg: Feedback message from the action server.
        """
        feedback: FollowJointTrajectory.Feedback = feedback_msg.feedback
        self.q = feedback.actual.positions
        self.qd = feedback.actual.velocities

    def _get_result_callback(self, future):
        """
        Log the action server's result and mark the operation as complete.

        :param future: Future object containing the result from the action server.
        """
        result: FollowJointTrajectory.Result = future.result().result
        self.get_logger().info(f'Goal completed with result: {result}')
        self._is_running = False

    def _goal_response_callback(self, future):
        """
        Handle the response from sending a goal to the action server.

        :param future: Future object containing the goal handle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._is_running = False
            fatal_msg = 'Gripper goal rejected!'
            self.get_logger().fatal(fatal_msg)
            raise RuntimeError(fatal_msg)

        self.get_logger().info('Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def send_goal(self, positions: list[float], velocities: list[float] = None, accelerations: list[float] = None, duration_s: float = 2.0):
        """
        Send a trajectory goal to the arm_controller.

        :param positions: Target joint positions (radians) for the arm.
        :param duration_s: Time in seconds to reach the target position.
        :return: Future object for the goal request.
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        if velocities is not None:
            point.velocities = velocities
        if accelerations is not None:
            point.accelerations = accelerations
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int((duration_s % 1) * S_TO_NS)
        goal_msg.trajectory.points.append(point)

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

        self._is_running = True
        future.add_done_callback(self._goal_response_callback)

        return future
    
    def send_trajectory(self, positions: np.ndarray, velocities: np.ndarray=None, accelerations: np.ndarray=None, duration_s: float = 0.05):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        traj_len = len(positions)
        for k in range(traj_len):
            traj_pt = JointTrajectoryPoint()
            traj_pt.positions = positions[k].tolist()
            if velocities is not None:
                traj_pt.velocities = velocities[k].tolist()
            if accelerations is not None:
                traj_pt.accelerations = accelerations[k].tolist()
            
            t_from_start = (k + 1) * duration_s
            sec = int(t_from_start)
            nsec = int((t_from_start - sec) * S_TO_NS)
            traj_pt.time_from_start.sec = sec
            traj_pt.time_from_start.nanosec = nsec

            goal_msg.trajectory.points.append(traj_pt)
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._is_running = True
        future.add_done_callback(self._goal_response_callback)

        return future



def main():
    rclpy.init()
    ball_prediction_node = BallPredictionNode()
    arm_node = ArmNode(action_name='arm_controller/follow_joint_trajectory')
    arm_state_node = ArmStateNode()
    mpc_log_node = MpcLogNode()
    arm_model = AlohaArmCasadi(free_joint_expr='follower_left_joint_[0-5]', ee_frame_name='follower_left_link_6')

    last_traj_q = None
    last_traj_qd = None
    last_traj_qdd = None
    last_mpc_error = None

    traj_end_time = None

    def send_cmd(q: np.ndarray, qd: np.ndarray=None, qdd: np.ndarray=None, dt=2.):
        print('sending', q, qd, qdd, dt)
        future = arm_node.send_goal(q.tolist(), 
                                    qd.tolist() if qd is not None else None,
                                    qdd.tolist() if qdd is not None else None,
                                    duration_s=dt)
        try:
            rclpy.spin_until_future_complete(arm_node, future)
        except:
            return False
        return False
    
    def send_trajectory(q: np.ndarray, qd: np.ndarray=None, qdd: np.ndarray=None, dt=0.05):
        print('sending traj')
        print('traj q', q.min(), q.max())
        if qd is not None:
            print('traj qd', qd.min(), qd.max())
        if qdd is not None:
            print('traj qdd', qdd.min(), qd.max())
        future = arm_node.send_trajectory(q, qd, qdd, duration_s=dt)
        try:
            rclpy.spin_until_future_complete(arm_node, future)
        except:
            return False
        return False

    t = time.time()
    v_des = np.array([0., 0., 0.])
    n_des = np.array([1., 0., 0.])
    q_sol, _, _, _, success, _ = arm_model.solve_ocp(
        p_des = np.array([hit_plane, 0., table_surface_z + 0.18]),
        v_des=v_des,
        o_des=n_des,
        q0=np.zeros(6, dtype=float),
        qd0=np.zeros(6, dtype=float),
        t=t,
        t_f=t + 1.
    )
    home_position = q_sol[-1]
    if success:
        send_cmd(q_sol[-1])
    else:
        print('init fail')

    q_sol, qd_sol, qdd_sol, idx = None, None, None, 0
    last_sent = None
    while True:
        t = time.time()

        rclpy.spin_once(arm_state_node, timeout_sec=0.01)
        rclpy.spin_once(ball_prediction_node, timeout_sec=0.01)

        mpc_time_loop = None
        if ball_prediction_node.ref_t is not None:
            mpc_time_loop = time.time() - ball_prediction_node.ref_t

        mpc_log_node.publish_step(
            build_mpc_step_msg(
                mpc_log_node,
                t,
                ball_prediction_node.pos,
                ball_prediction_node.vel,
                ball_prediction_node.p_des,
                ball_prediction_node.t_strike,
                last_traj_q,
                last_traj_qd,
                last_traj_qdd,
                arm_state_node.q,
                arm_state_node.qd,
                mpc_time_loop,
                last_mpc_error,
            )
        )

        if arm_state_node.q is None or arm_state_node.qd is None:
            continue

        if traj_end_time is not None and t - traj_end_time > 0.5:
            traj_end_time = None
            send_cmd(home_position)
        if ball_prediction_node.p_des is None or ball_prediction_node.t_strike is None:
            continue
        # step_data['p_des'] = ball_prediction_node.p_des
        # step_data['t_strike'] = ball_prediction_node.t_strike
        
        if abs(ball_prediction_node.p_des[0] - hit_plane) > 1e-2:
            # print('no reach', abs(ball_prediction_node.p_des[0] - hit_plane))
            continue


        # print('state', arm_state_node.q, arm_state_node)

        if ball_prediction_node.t_strike <= t:
            continue
        
        if not ball_prediction_node.has_ball:
            arm_model.reset_solver()
            continue

        if last_sent is not None and np.linalg.norm(last_sent - ball_prediction_node.p_des) <= 0.03:
            continue

        if ball_prediction_node.t_strike <= 0.05:
            continue
        
        start_time = time.time()
        v_des, n_des = ball_prediction_node.get_return_params()
        if v_des is None or n_des is None:
            v_des = np.array([0., 0., 0.])
            n_des = np.array([1., 0., 0.])
        print('v_des', v_des)
        print('n_des', n_des)
        q_sol, qd_sol, qdd_sol, _, success, idx = arm_model.solve_ocp(
            p_des=ball_prediction_node.p_des,
            v_des=v_des,
            o_des=n_des,
            q0=arm_state_node.q,
            qd0=arm_state_node.qd,
            t=t,
            t_f=ball_prediction_node.t_strike,
        )
        qdd_sol = np.concatenate([qdd_sol, np.zeros_like(qdd_sol[-1:])], axis=0)

        if not success:
            print('mpc fail')
            continue
        else:
            print('mpc success')
        
        if q_sol is None:
            continue

        idx += 1
        q_sol, qd_sol, qdd_sol = q_sol[idx:], qd_sol[idx:], qdd_sol[idx:]
        
        # step_data['q_traj'] = q_sol
        # step_data['qd_traj'] = qd_sol
        # step_data['qdd_traj'] = qdd_sol
        
        
        fk = arm_model.fk_numeric(q_sol[-1])
        error = ball_prediction_node.p_des - fk
        error_norm = np.linalg.norm(ball_prediction_node.p_des - fk)

        if error_norm > 0.06:
            continue

        print('error', error, error_norm)

        # data.append((ball_prediction_node.p_des, arm_state_node.q, q_sol, qd_sol, qdd_sol, idx, t))
        # if len(data) - last_save >= 10:
        #     last_save = len(data)
        #     with open('mpc_data.pkl', 'wb') as f:
        #         pickle.dump(data, f)


        elapsed_time = time.time() - ball_prediction_node.ref_t
        print('elapsed time', elapsed_time)
        if elapsed_time < compute_time_adjust:
            time.sleep(max(compute_time_adjust - elapsed_time - 0.06, 0))
        last_traj_q = q_sol
        last_traj_qd = qd_sol
        last_traj_qdd = qdd_sol
        last_mpc_error = error_norm
        send_trajectory(q_sol, qd_sol, qdd_sol)
        # print('last joint', q_sol[-1][-1], arm_state_node.q[-1])
        last_sent = ball_prediction_node.p_des
        traj_end_time = ball_prediction_node.t_strike




if __name__ == '__main__':
    main()