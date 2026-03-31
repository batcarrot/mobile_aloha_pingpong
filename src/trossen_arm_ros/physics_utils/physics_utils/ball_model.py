import numpy as np
import matplotlib.pyplot as plt
import numba as nb

try:
    import casadi as cs
except ImportError:
    print("CasADi not found")

from physics_utils.common import (
    table_x, table_length, table_width, table_surface_z,
    hit_plane, in_table, cam_z_error
)

try:
    from physics_utils.ocp import OCPSolver
except ImportError:
    print("OCPSolver not available")


def normalize_casadi(v, eps=1e-9):
    return v / (cs.norm_2(v) + eps)


def _in_table_bounds(pos, x_min, x_max, y_min, y_max, z_min):
    # pos: (3,)
    return (x_min <= pos[0] <= x_max) and (y_min <= pos[1] <= y_max) and (pos[2] >= z_min)


@nb.njit(cache=True, fastmath=True)
def _predict_numba(
    pos_init, vel_init, ang_vel_init,
    dt, stop_x, stop_t, table_edge,
    friction_coeff, restitution_coeff, k_D, ball_radius,
    table_z,
    x_min, x_max, y_min, y_max,
    g=9.81,
    max_steps=2_000_000,
    eps=1e-12,
    bounce_cooldown=10,
):
    pos = pos_init.copy()
    vel = vel_init.copy()
    ang = ang_vel_init.copy()

    t = 0.0
    step = 0
    bounce_n = -100
    n = 0

    while (pos[0] > stop_x) and (y_min <= pos[1] <= y_max):
        step += 1
        t += dt
        if t > stop_t:
            break
        n += 1
        if step > max_steps:
            break

        v_n = np.linalg.norm(vel)
        accx = -k_D * v_n * vel[0]
        accy = -k_D * v_n * vel[1]
        accz = -k_D * v_n * vel[2] - g

        # table contact/impact update (instantaneous)
        if (pos[2] - ball_radius) <= table_z and pos[0] >= table_edge and n - bounce_n > bounce_cooldown:
            bounce_n = n
            # v_tangent = [vx - wy*r, vy + wx*r, 0]
            vtx = vel[0] - ang[1] * ball_radius
            vty = vel[1] + ang[0] * ball_radius
            vtn = (vtx * vtx + vty * vty) ** 0.5

            # avoid division by zero if v_tangent ~ 0
            if vtn < eps:
                vtn = eps

            abs_vz = vel[2] if vel[2] >= 0.0 else -vel[2]
            k = friction_coeff * (1.0 + restitution_coeff) * abs_vz / vtn

            contact = 1.0 - (2.0 / 5.0) * k
            alpha_sliding = k
            alpha_rolling = 2.0 / 5.0
            alpha = alpha_sliding if contact > 0.0 else alpha_rolling

            # Avoid building matrices; expand the matmuls explicitly
            # A_lin @ vel + B_lin @ ang
            # A_lin: diag([1-alpha, 1-alpha, -e])
            # B_lin: [[0, alpha*r, 0], [-alpha*r, 0, 0], [0,0,0]]
            ar = alpha * ball_radius
            one_ma = 1.0 - alpha
            e = restitution_coeff

            velx = one_ma * vel[0] + ar * ang[1]
            vely = one_ma * vel[1] - ar * ang[0]
            velz = -e * vel[2]

            # A_ang @ vel + B_ang @ ang
            # A_ang: [[0, -3a/(2r), 0], [3a/(2r), 0, 0], [0,0,0]]
            # B_ang: diag([1-3a/2, 1-3a/2, 1])
            c = (3.0 * alpha) / (2.0 * ball_radius)
            b = 1.0 - 1.5 * alpha

            angx = (-c) * vel[1] + b * ang[0]
            angy = ( c) * vel[0] + b * ang[1]
            angz = ang[2]

            vel[0], vel[1], vel[2] = velx, vely, velz
            ang[0], ang[1], ang[2] = angx, angy, angz

        # gravity + integrate
        vel[0] += accx * dt
        vel[1] += accy * dt
        vel[2] += accz * dt

        pos[0] += vel[0] * dt
        pos[1] += vel[1] * dt
        pos[2] += vel[2] * dt

    return pos, vel, t


class BallModel:
    def __init__(self, friction_coeff=0.25, restitution_coeff=0.9, ball_radius=0.02, k_M=0.0036, k_D=0.12, g=9.81):
        self.friction_coeff = friction_coeff
        self.restitution_coeff = restitution_coeff
        self.ball_radius = ball_radius
        self.k_M = k_M
        self.k_D = k_D
        self.g = g
        
        self.m = 0.0027
        self.I = 2/5 * self.m * self.ball_radius ** 2
    
    def get_full_traj(self, pos_init, vel_init, ang_vel_init, dt=0.001, stop_x=hit_plane, stop_t=None, stop_n=10000., table_z=table_surface_z):
        pos_cur = pos_init.copy()
        vel_cur = vel_init.copy()
        ang_vel_cur = ang_vel_init.copy()
        
        traj_pos = []
        traj_vel = []
        traj_ang_vel = []
        traj_time = []
        
        t = 0
        n = 0
        bounce_n = 0
        while (stop_x is None or pos_cur[0] > stop_x) and (stop_t is None or t <= stop_t) and n < stop_n:
            traj_pos.append(pos_cur.copy())
            traj_vel.append(vel_cur.copy())
            traj_ang_vel.append(ang_vel_cur.copy())
            traj_time.append(t)
            n += 1

            t += dt
            if stop_t is not None and t > stop_t:
                break
            
            acc = -self.k_D * np.linalg.norm(vel_cur) * vel_cur + self.k_M * np.cross(ang_vel_cur, vel_cur) + np.array([0, 0, -self.g])
            
            if pos_cur[2] - self.ball_radius <= table_z and n - bounce_n > 10:
                bounce_n = n
                v_tangent = np.array([vel_cur[0] - ang_vel_cur[1] * self.ball_radius, 
                                      vel_cur[1] + ang_vel_cur[0] * self.ball_radius, 
                                      0])
                contact = 1 - 2 / 5 * self.friction_coeff * (1 + self.restitution_coeff) * np.abs(vel_cur[2]) / np.linalg.norm(v_tangent)
                alpha_sliding = self.friction_coeff * (1 + self.restitution_coeff) * np.abs(vel_cur[2]) / np.linalg.norm(v_tangent)
                alpha_rolling = 2 / 5
                alpha = alpha_sliding if contact > 0 else alpha_rolling
                
                A_lin = np.array([[1 - alpha, 0, 0],
                                  [0, 1 - alpha, 0],
                                  [0, 0, -self.restitution_coeff]])
                B_lin = np.array([[0, alpha * self.ball_radius, 0],
                                  [-alpha * self.ball_radius, 0, 0],
                                  [0, 0, 0]])
                A_ang = np.array([[0, -3 * alpha / (2 * self.ball_radius), 0],
                                  [3 * alpha / (2 * self.ball_radius), 0, 0],
                                  [0, 0, 0]])
                B_ang = np.array([[1 - 3 * alpha / 2, 0, 0],
                                  [0, 1 - 3 * alpha / 2, 0],
                                  [0, 0, 1]])
                
                vel_post = A_lin @ vel_cur + B_lin @ ang_vel_cur
                ang_vel_post = A_ang @ vel_cur + B_ang @ ang_vel_cur
                vel_cur = vel_post
                ang_vel_cur = ang_vel_post
            vel_cur += acc * dt
            pos_cur += vel_cur * dt
        traj_pos.append(pos_cur.copy())
        traj_vel.append(vel_cur.copy())
        traj_ang_vel.append(ang_vel_cur.copy())
        traj_time.append(t)
        return np.array(traj_pos), np.array(traj_vel), np.array(traj_ang_vel), np.array(traj_time)
    
    def predict(self, pos_init, vel_init, ang_vel_init, dt=0.001, stop_x=hit_plane, use_numba=True, stop_t=None, return_vel=False):
        if use_numba:
            if stop_t is None:
                stop_t = 1000.0
            p, v, t = _predict_numba(
                pos_init, vel_init, ang_vel_init,
                dt, stop_x, stop_t, table_x - table_length / 2,
                self.friction_coeff, self.restitution_coeff, self.k_D, self.ball_radius,
                table_surface_z + cam_z_error,
                table_x - table_length / 2, table_x + table_length / 2,
                -table_width / 2, table_width / 2,
            )
            # p += np.array([0., -0.04, 0.05])
            if return_vel:
                return p, v, t
            else:
                return p, t
        pos_cur = pos_init.copy()
        vel_cur = vel_init.copy()
        ang_vel_cur = ang_vel_init.copy()
        
        t = 0
        while pos_cur[0] > stop_x and in_table(pos_cur):
            t += dt
            
            acc = np.array([0, 0, -9.81])
            
            if pos_cur[2] - self.ball_radius <= table_surface_z:
                v_tangent = np.array([vel_cur[0] - ang_vel_cur[1] * self.ball_radius, 
                                      vel_cur[1] + ang_vel_cur[0] * self.ball_radius, 
                                      0])
                contact = 1 - 2 / 5 * self.friction_coeff * (1 + self.restitution_coeff) * np.abs(vel_cur[2]) / np.linalg.norm(v_tangent)
                alpha_sliding = self.friction_coeff * (1 + self.restitution_coeff) * np.abs(vel_cur[2]) / np.linalg.norm(v_tangent)
                alpha_rolling = 2 / 5
                alpha = alpha_sliding if contact > 0 else alpha_rolling
                
                A_lin = np.array([[1 - alpha, 0, 0],
                                  [0, 1 - alpha, 0],
                                  [0, 0, -self.restitution_coeff]])
                B_lin = np.array([[0, alpha * self.ball_radius, 0],
                                  [-alpha * self.ball_radius, 0, 0],
                                  [0, 0, 0]])
                A_ang = np.array([[0, -3 * alpha / (2 * self.ball_radius), 0],
                                  [3 * alpha / (2 * self.ball_radius), 0, 0],
                                  [0, 0, 0]])
                B_ang = np.array([[1 - 3 * alpha / 2, 0, 0],
                                  [0, 1 - 3 * alpha / 2, 0],
                                  [0, 0, 1]])
                
                vel_post = A_lin @ vel_cur + B_lin @ ang_vel_cur
                ang_vel_post = A_ang @ vel_cur + B_ang @ ang_vel_cur
                vel_cur = vel_post
                ang_vel_cur = ang_vel_post
            vel_cur += acc * dt
            pos_cur += vel_cur * dt
        
        return pos_cur, t

    def _get_racket_frame(self, normal):
        z = normalize_casadi(normal)
        ref1 = cs.DM([1.0, 0.0, 0.0])
        ref2 = cs.DM([0.0, 1.0, 0.0])
        dot1 = cs.fabs(cs.dot(z, ref1))
        ref = cs.if_else(dot1 > 0.9, ref2, ref1)

        x = normalize_casadi(cs.cross(ref, z))
        y = cs.cross(z, x)

        R_WR = cs.horzcat(x, y, z)
        return R_WR
    
    def f_contact(self, 
                  vel_pre_impact,
                  ang_vel_pre_impact,
                  v_paddle,
                  n_paddle,
                  restitution_coeff=None,
                  k_p=1e-5):
        restitution_coeff = self.restitution_coeff if restitution_coeff is None else restitution_coeff


        Kv = k_p / self.m
        Kw = k_p / self.I
        
        R_WR = self._get_racket_frame(n_paddle)
        R_RW = R_WR.T
        
        v_rel_W = vel_pre_impact - v_paddle
        v_pre_R = R_RW @ v_rel_W
        w_pre_R = R_RW @ ang_vel_pre_impact
        
        Av = cs.DM([
            [1 - Kv, 0, 0],
            [0, 1 - Kv, 0],
            [0, 0, -restitution_coeff]
        ])
        Bv = Kv * cs.DM([
            [0, self.ball_radius, 0],
            [-self.ball_radius, 0, 0],
            [0, 0, 0]
        ])
        Aw = Kw * cs.DM([
            [0, -self.ball_radius, 0],
            [self.ball_radius, 0, 0],
            [0, 0, 0]
        ])
        Bw = cs.DM([
            [1 - Kw * self.ball_radius ** 2, 0, 0],
            [0, 1 - Kw * self.ball_radius ** 2, 0],
            [0, 0, 1]
        ])
        
        v_post_R = Av @ v_pre_R + Bv @ w_pre_R
        w_post_R = Aw @ v_pre_R + Bw @ w_pre_R
        
        v_post_W = R_WR @ v_post_R + v_paddle
        w_post_W = R_WR @ w_post_R
        return v_post_W, w_post_W
    
    def solve_landing(self,
                  p_des: np.ndarray, p_land: np.ndarray,
                  vel_pre_impact: np.ndarray, ang_vel_pre_impact: np.ndarray,
                  k_p=1e-6,
                  w_v=1.0,
                  w_t=1.0,
                  table_surface_z=table_surface_z,
                  t_max=2.0):
        """
        Solve for paddle velocity v_des, plane normal n_des, and time-to-table t_hit,
        such that the *first* table contact (z = table_surface_z) occurs at p_land.

        This version:
        - uses t_hit as a decision variable (no sqrt)
        - has no rollout loop
        - does NOT redefine N from dt (fixes your old confusion)
        """

        g_W = np.array([0.0, 0.0, -9.81], dtype=float)

        # Decision variables
        v_des = cs.MX.sym('v_des', 3)
        n_des = cs.MX.sym('n_des', 3)
        t_hit = cs.MX.sym('t_hit', 1)

        # Post-impact ball velocities (from your contact model)
        v_post, w_post = self.f_contact(
            vel_pre_impact=cs.MX(vel_pre_impact),
            ang_vel_pre_impact=cs.MX(ang_vel_pre_impact),
            v_paddle=v_des,
            n_paddle=n_des,
            k_p=k_p,
            restitution_coeff=0.5
        )

        # Closed-form position at time t_hit (continuous ballistic, no aero)
        p0 = cs.MX(p_des)
        g  = cs.MX(g_W)

        pos_hit = p0 + v_post * t_hit + 0.5 * g * (t_hit**2)

        # Build NLP
        ocp_solver = OCPSolver()

        # --- Constraints ---
        # Unit normal
        ocp_solver.add_eq_constraint(cs.sumsqr(n_des), 1.0)

        # Time bounds: 0 <= t_hit <= t_max (prevents crazy solutions)
        ocp_solver.add_geq_constraint(t_hit, 0.0)
        ocp_solver.add_leq_constraint(t_hit, t_max)

        # Table plane contact (this ensures contact occurs on the table plane)
        ocp_solver.add_eq_constraint(pos_hit[2], table_surface_z)

        # Landing XY exactly at desired location
        ocp_solver.add_leq_constraint(cs.sumsqr(pos_hit[0:2] - cs.MX(p_land[0:2])), 1e-4)  # Allow some small tolerance

        v_rel_W = cs.MX(vel_pre_impact) - v_des
        ocp_solver.add_leq_constraint(cs.dot(n_des, v_rel_W), -1e-3)

        # --- Cost ---
        ocp_solver.add_cost(w_v * cs.sumsqr(v_des))
        ocp_solver.add_cost(w_t * t_hit)  # Encourage shorter time to hit (optional)

        # Initial guess (v_des, n_des, t_hit)
        x0 = [
            0.0, 0.0, 0.0,   # v_des
            1.0, 0.0, 0.0,   # n_des
            0.1              # t_hit
        ]

        # Solve: IMPORTANT — pass t_hit too
        sol, (success, _cost) = ocp_solver.build_and_solve(
            x0,
            v_des,
            n_des,
            t_hit
        )

        if not success:
            return None, None

        v_des_opt = sol[0:3]
        n_des_opt = sol[3:6] / np.linalg.norm(sol[3:6])  # Ensure normal is unit length
        t_hit_opt = sol[6]
        
        return np.array(v_des_opt), np.array(n_des_opt)
    
    def get_return_vel(self, pos_strike, vel_strike, ang_vel_strike):
        # Simple heuristic: reverse x-velocity and add some extra speed based on incoming speed
        return np.array([0.05, 0., 0.])
