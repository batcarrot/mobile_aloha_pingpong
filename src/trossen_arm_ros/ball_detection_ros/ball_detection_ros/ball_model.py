import numpy as np

class BallStateEstimatorNoSpin:
    def __init__(self, pos_list, t_list, g=-9.81, k_D=0.12, phys_weight=0.05, irls_iters=3):
        self.g = g
        self.k_D = k_D
        self.phys_weight = phys_weight
        self.irls_iters = irls_iters

        pos_arr = np.array(pos_list).reshape(-1, 3)
        t_arr   = np.array(t_list).reshape(-1, 1)
        self.t0 = t_arr[0, 0]
        self.t_list   = t_arr - self.t0
        self.pos_list = pos_arr

        # Bootstrap: no-drag linear fit as initial guess
        self.coeffs = self._fit_no_drag()
        self.coeffs = self._irls(self.coeffs)

    # ------------------------------------------------------------------
    # Bootstrap — exact no-drag solution as warm start
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # IRLS — linearize drag around current velocity, solve, repeat
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Incremental update — RLS for position rows + re-run IRLS
    # ------------------------------------------------------------------

    def add_point(self, t_new, pos_new):
        pos_new = np.asarray(pos_new, dtype=float).reshape(3)
        tau = float(t_new) - self.t0
        self.t_list   = np.vstack([self.t_list,  [[tau]]])
        self.pos_list = np.vstack([self.pos_list, pos_new])
        # Warm-start IRLS from previous solution — converges in 2-3 iters
        self.coeffs = self._irls(self.coeffs)

    # ------------------------------------------------------------------
    # Polynomial helpers
    # ------------------------------------------------------------------

    def _P(self, coeffs, t):
        return coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3

    def _dP(self, coeffs, t):
        return coeffs[1] + 2.0*coeffs[2]*t + 3.0*coeffs[3]*t**2

    def _ddP(self, coeffs, t):
        return 2.0*coeffs[2] + 6.0*coeffs[3]*t

    # ------------------------------------------------------------------

    def predict(self, t):
        t = np.asarray(t, dtype=float).reshape(-1, 1) - self.t0
        return self._P(self.coeffs, t), self._dP(self.coeffs, t), np.zeros(3)  # zero spin estimate