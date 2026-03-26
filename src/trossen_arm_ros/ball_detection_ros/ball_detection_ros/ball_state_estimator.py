import numpy as np
from physics_utils import BallModel
from physics_utils.common import *

class BallStateEstimatorNoSpin:
    def __init__(self, pos_list=[], t_list=[], g=-9.81, k_D=0.1487858280740126, phys_weight=0.05, irls_iters=3, init_points=40):
        self.g = g
        self.k_D = k_D
        self.phys_weight = phys_weight
        self.irls_iters = irls_iters
        self.init_points = init_points
        
        self.ball_model = BallModel(k_D=k_D, friction_coeff=0.05, restitution_coeff=0.93)
        self.reset()

    def _fit_no_drag(self):
        """Closed-form no-drag fit (c2, c3 fixed)."""
        c2 = np.array([0.0, 0.0, self.g / 2.0])
        A  = np.column_stack([np.ones_like(self.t_list), self.t_list])
        b  = self.pos_list - c2 * self.t_list**2
        x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        # Assemble full (4,3) coeffs
        coeffs = np.zeros((4, 3))
        coeffs[0] = x[0]
        coeffs[1] = x[1]
        coeffs[2] = c2
        return coeffs

    def _irls(self, coeffs):
        t = self.t_list  # (N,1)
        N = len(t)

        for _ in range(self.irls_iters):
            dP = self._dP(coeffs, t)                          # (N,3) current vel estimate
            dP_norm = np.linalg.norm(dP, axis=1)              # (N,)
            # Linearized drag acceleration at each timestep: -k_D * ||dP|| * dP
            drag_acc = -self.k_D * dP_norm[:, None] * dP      # (N,3)
            # Effective rhs for P''(t): gravity + drag (treated as known)
            eff_acc = np.array([0.0, 0.0, self.g]) + drag_acc # (N,3)

            # Build combined system:
            #   [    A_pos         ] [coeffs]   [pos_list          ]
            #   [phys_w * A_phys   ]          = [phys_w * eff_acc  ]
            #
            # A_pos  rows: [1, t, t^2, t^3]
            # A_phys rows: [0, 0,  2,  6t ]  (second derivative operator)

            A_pos  = np.column_stack([
                np.ones_like(t), t, t**2, t**3
            ])  # (N,4)

            A_phys = np.column_stack([
                np.zeros_like(t),
                np.zeros_like(t),
                np.full_like(t, 2.0),
                6.0 * t,
            ])  # (N,4)

            A = np.vstack([A_pos, self.phys_weight * A_phys])          # (2N,4)
            b = np.vstack([self.pos_list, self.phys_weight * eff_acc])  # (2N,3)

            x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            coeffs = x  # (4,3)

        return coeffs

    def reset(self):
        self.pos_list = np.zeros((0, 3))
        self.t_list = np.zeros((0, 1))
        self.init = False
        self.state = None
        self.bounce_t = None
        
    def add_point(self, t_new, pos_new):
        if len(self.t_list) == 0:
            self.t0 = float(t_new)
        # BOUNCE
        if pos_new[2] <= table_surface_z + 0.05:
            print('BOUNCE')
            if self.bounce_t is None:
                self.bounce_t = float(t_new)
                self.bounce_state = self.state
        
            self.pos_list = np.zeros((0, 3))
            self.t_list = np.zeros((0, 1))
            self.init = False    
        
        pos_new = np.asarray(pos_new, dtype=float).reshape(3)
        tau = float(t_new) - self.t0
        self.t_list   = np.vstack([self.t_list,  [[tau]]])
        self.pos_list = np.vstack([self.pos_list, pos_new])
        
        if len(self.t_list) < self.init_points:
            if self.bounce_t is not None and self.state is not None:
                print('using bounce', self.state[0])
                p, v, _ = self.ball_model.predict(
                    self.bounce_state[0], 
                    self.bounce_state[1], 
                    np.zeros(3), 
                    stop_t=t_new - self.bounce_t, 
                    return_vel=True,
                )
                self.bounce_t = t_new
                self.bounce_state = [p, v]
                return p, v, np.zeros(3)
            return None

        self.bounce_t = None
        self.bounce_state = None

        if not self.init:
            self.coeffs = self._fit_no_drag()
            self.init = True
        
        # Warm-start IRLS from previous solution — converges in 2-3 iters
        self.coeffs = self._irls(self.coeffs)
        self.update_state()
        
        
        return self.state

    def update_state(self):
        self.state = [self._P(self.coeffs, self.t_list[-1]), self._dP(self.coeffs, self.t_list[-1])]

    def _P(self, coeffs, t):
        return coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3

    def _dP(self, coeffs, t):
        return coeffs[1] + 2.0*coeffs[2]*t + 3.0*coeffs[3]*t**2

    def _ddP(self, coeffs, t):
        return 2.0*coeffs[2] + 6.0*coeffs[3]*t

    def predict(self, t):
        if not self.init:
            return False
        t = np.asarray(t, dtype=float).reshape(-1, 1) - self.t0
        return self._P(self.coeffs, t), self._dP(self.coeffs, t), np.zeros(3)
