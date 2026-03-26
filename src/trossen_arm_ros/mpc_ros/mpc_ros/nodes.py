import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.constants import S_TO_NS
from rclpy.node import Node

from ball_state_msgs.msg import BallState
from mpc_ros import BallModel
from physics_utils.common import *

import pickle


class BallPredictionNode(Node):
    def __init__(self):
        super().__init__("ball_prediction_node")
        self.p_des: Optional[np.ndarray] = None
        self.t_strike: Optional[float] = None
        self.has_ball: bool = False

        k_D = 0.1487858280740126
        self.ball_model = BallModel(
            k_D=k_D, friction_coeff=0.08, restitution_coeff=0.9
        )
        self.ema_strength = 0.
        self.filtered_p_des = None
        self.data = []
        self.last_save = 0

        self.create_subscription(
            BallState,
            "ball_position_node/ball_state",
            self._cb_ball_pos,
            1,
        )
        self._pub_pos = self.create_publisher(
            PointStamped, "~/pred_ball_pos", 1
        )
        self.pos, self.vel = None, None
        self.ref_t = None

    def _cb_ball_pos(self, msg: BallState):
        self.has_ball = msg.has_ball
        if not self.has_ball:
            self.pos, self.vel = None, None
            self.filtered_p_des = None
            return
        pos = np.array(
            [msg.position.x, msg.position.y, msg.position.z],
            dtype=float,
        )
        # if abs(pos[0] - hit_plane) <= 
        vel = np.array(
            [msg.velocity.x, msg.velocity.y, msg.velocity.z],
            dtype=float,
        )
        self.pos, self.vel = pos, vel
        self.ref_t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.p_des, self.v_des, self.t_strike = self.ball_model.predict(
            pos,
            vel,
            np.zeros(3, dtype=np.float32),
            return_vel=True
        )
        if self.filtered_p_des is None:
            self.filtered_p_des = self.p_des
        else:
            a = self.ema_strength
            self.filtered_p_des = a * self.filtered_p_des + (1 - a) * self.p_des

            

        t = time.time()

        self.data.append((pos, vel, self.p_des, self.t_strike, t))
        if len(self.data) - self.last_save >= 50:
            self.last_save = len(self.data)
            with open('pred_data.pkl', 'wb') as f:
                pickle.dump(self.data, f)

        sec = int(t)
        out = PointStamped()
        out.header.stamp.sec = sec
        out.header.stamp.nanosec = sec * S_TO_NS
        out.header.frame_id = "map"
        out.point.x = float(self.filtered_p_des[0])
        out.point.y = float(self.filtered_p_des[1])
        out.point.z = float(self.filtered_p_des[2])
        self._pub_pos.publish(out)

        self.t_strike += self.ref_t - compute_time_adjust

    def get_return_params(self):
        return self.ball_model.solve_landing(
            self.p_des,
            np.array([table_x + table_length / 2, 0, table_surface_z]),
            self.v_des,
            np.zeros(3),
        )

def main():
    rclpy.init()
    node = BallPredictionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


