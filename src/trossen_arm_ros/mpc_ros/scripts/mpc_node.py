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

from mpc_ros import (
    BallModel,
    AlohaArmCasadi,
    hit_plane, table_surface_z
)

from rclpy.action import ActionClient
from rclpy.constants import S_TO_NS
from trajectory_msgs.msg import JointTrajectoryPoint
from mpc_ros.nodes import BallPredictionNode

class ArmStateNode(Node):
    def __init__(self):
        super().__init__('arm_state_node')
        self.q = None
        self.qd = None

        self.create_subscription(JointTrajectoryControllerState, '/arm_controller/controller_state', self._cb_ctrl_state, 10)
    
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

def main():
    rclpy.init()
    ball_prediction_node = BallPredictionNode()
    arm_node = ArmNode(action_name='arm_controller/follow_joint_trajectory')
    arm_state_node = ArmStateNode()
    arm_model = AlohaArmCasadi(free_joint_expr='follower_left_joint_[0-5]', ee_frame_name='follower_left_link_6')

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

    home_position = np.zeros(6,)
    send_cmd(home_position)

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
    if success:
        send_cmd(q_sol[-1])
    else:
        print('init fail')

    q_sol, qd_sol, qdd_sol, idx = None, None, None, 0
    last_mpc_time = 0.
    last_sent = None
    while True:
        t = time.time()
        rclpy.spin_once(ball_prediction_node, timeout_sec=0.01)
        if ball_prediction_node.p_des is None or ball_prediction_node.t_strike is None:
            continue
        
        if abs(ball_prediction_node.p_des[0] - hit_plane) > 1e-2:
            print('no reach', abs(ball_prediction_node.p_des[0] - hit_plane))
            continue

        rclpy.spin_once(arm_state_node, timeout_sec=0.01)
        if arm_state_node.q is None or arm_state_node.qd is None:
            continue

        print('state', arm_state_node.q, arm_state_node)

        if ball_prediction_node.t_strike <= t:
            continue
        

        t = time.time()
        if ball_prediction_node.has_ball and t - last_mpc_time >= 0.05:
            print(ball_prediction_node.p_des, ball_prediction_node.t_strike - t)
            q_sol, qd_sol, qdd_sol, _, success, idx = arm_model.solve_ocp(
                p_des=ball_prediction_node.p_des,
                v_des=v_des,
                o_des=n_des,
                q0=arm_state_node.q,
                qd0=arm_state_node.qd,
                t=t,
                t_f=ball_prediction_node.t_strike,
            )
            last_mpc_time = t

            if not success:
                print('mpc fail')
                continue
            else:
                print('mpc success')
        else:
            print('no ball')
            arm_model.reset_solver()

        
        if q_sol is None:
            continue

        jump_idx = 5

        t0 = ball_prediction_node.t_strike - 30 * 0.05
        idx = int(np.clip((t - t0) / 0.05, 0, 30))
        if idx >= len(q_sol) - jump_idx:
            continue
        next_idx = min(idx + jump_idx, len(qdd_sol) - 1)

        # np.savez('traj.npz', q=q_sol, qd=qd_sol, qdd=qdd_sol, idx=idx)

        print('time till hit', ball_prediction_node.t_strike - t)
        q_next = q_sol[idx]
        qd_next = qd_sol[idx]
        qdd_next = qdd_sol[idx]

        print('idx', idx)

        q_next = q_sol[-1]
        fk = arm_model.fk_numeric(q_next)
        error = ball_prediction_node.p_des - fk
        error_norm = np.linalg.norm(ball_prediction_node.p_des - fk)

        if error_norm > 0.1:
            continue
        print('error', error, error_norm)

        if last_sent is not None and np.linalg.norm(q_next - last_sent) < 1e-3:
            continue

        last_sent = q_next
        send_cmd(q_next, None, None, dt=(ball_prediction_node.t_strike - t) * 0.9)



if __name__ == '__main__':
    main()