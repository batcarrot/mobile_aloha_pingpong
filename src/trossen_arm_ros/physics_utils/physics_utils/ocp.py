
import casadi as cs

import numpy as np

class OCPSolver:
    """
    Build constraints/cost incrementally, then compile once via finalize(...),
    then solve many times via solve(...), optionally with warm-start multipliers
    and variable bounds.

    Consistent with calls like:
        ocp.finalize(x=x, p=p)
        x_opt, lam_x, lam_g, (success, cost) = ocp.solve(
            x0=x0, p=p_val, lam_x0=lam_x, lam_g0=lam_g
        )
    """
    def __init__(self):
        self.g = []
        self.lbg = []
        self.ubg = []
        self.cost = 0

        self._solver = None
        self._expects_p = False

        # optional bounds on decision vector x
        self.lbx = None
        self.ubx = None

    def _bind(self, expr, lb, ub):
        expr = cs.vec(expr)  # flatten to (m,1)
        m = int(expr.shape[0])

        self.g.append(expr)

        # allow scalar or list/np arrays
        if np.isscalar(lb):
            self.lbg.extend([float(lb)] * m)
        else:
            lb = list(lb)
            assert len(lb) == m
            self.lbg.extend([float(v) for v in lb])

        if np.isscalar(ub):
            self.ubg.extend([float(ub)] * m)
        else:
            ub = list(ub)
            assert len(ub) == m
            self.ubg.extend([float(v) for v in ub])

    def add_cost(self, expr):
        self.cost = self.cost + expr

    def add_eq_constraint(self, lhs, rhs):
        self._bind(lhs - rhs, 0.0, 0.0)

    def add_leq_constraint(self, lhs, rhs):
        # lhs <= rhs  <=>  lhs - rhs <= 0
        self._bind(lhs - rhs, -cs.inf, 0.0)

    def add_geq_constraint(self, lhs, rhs):
        # lhs >= rhs  <=>  lhs - rhs >= 0
        self._bind(lhs - rhs, 0.0, cs.inf)

    def set_x_bounds(self, lbx, ubx):
        """Set bounds on the decision vector x (recommended for joint limits)."""
        self.lbx = np.asarray(lbx, dtype=float).reshape(-1)
        self.ubx = np.asarray(ubx, dtype=float).reshape(-1)
        assert self.lbx.shape == self.ubx.shape

    def finalize(self, x, p=None, ipopt_opts=None):
        """
        Compile the NLP solver once.

        Args:
            x: cs.SX/MX decision vector (column)
            p: cs.SX/MX parameter vector (column) or None
        """
        g_expr = cs.vertcat(*self.g) if len(self.g) else cs.DM.zeros(0, 1)

        nlp = {"x": x, "f": self.cost, "g": g_expr}
        if p is not None:
            nlp["p"] = p
            self._expects_p = True
        else:
            self._expects_p = False

        ipopt = {
            "max_iter": 80,
            "print_level": 0,
            "sb": "yes",
            "tol": 1e-4,
            "acceptable_tol": 1e-3,
            "acceptable_iter": 5,
            "linear_solver": "mumps",

            # warm start (effective when reusing this compiled solver)
            "warm_start_init_point": "yes",
            "warm_start_bound_push": 1e-9,
            "warm_start_slack_bound_push": 1e-9,
            "warm_start_mult_bound_push": 1e-9,
            "mu_strategy": "adaptive",
        }
        if ipopt_opts:
            ipopt.update(ipopt_opts)

        self._solver = cs.nlpsol(
            "solver", "ipopt", nlp,
            {"ipopt": ipopt, "print_time": False}
        )

    def solve(self, x0, p=None, lam_x0=None, lam_g0=None):
        """
        Solve the compiled NLP.

        Returns:
            x_opt: np.ndarray (n_decision,)
            lam_x: np.ndarray (n_decision,)  (multipliers for bounds)
            lam_g: np.ndarray (n_constraints,) (multipliers for constraints)
            (success, cost): (bool, float)
        """
        assert self._solver is not None, "Call finalize(...) before solve(...)."

        kwargs = {
            "x0": cs.DM(np.asarray(x0, dtype=float).reshape(-1)),
            "lbg": cs.DM(np.asarray(self.lbg, dtype=float).reshape(-1)),
            "ubg": cs.DM(np.asarray(self.ubg, dtype=float).reshape(-1)),
        }

        # variable bounds (optional)
        if self.lbx is not None:
            kwargs["lbx"] = cs.DM(self.lbx)
            kwargs["ubx"] = cs.DM(self.ubx)

        # parameters (if used)
        if self._expects_p:
            assert p is not None, "This solver expects parameter vector p."
            kwargs["p"] = cs.DM(np.asarray(p, dtype=float).reshape(-1))

        # warm-start duals (optional)
        if lam_x0 is not None:
            kwargs["lam_x0"] = cs.DM(np.asarray(lam_x0, dtype=float).reshape(-1))
        if lam_g0 is not None:
            kwargs["lam_g0"] = cs.DM(np.asarray(lam_g0, dtype=float).reshape(-1))

        sol = self._solver(**kwargs)
        stats = self._solver.stats()

        x_opt = np.asarray(sol["x"]).reshape(-1)
        lam_x = np.asarray(sol["lam_x"]).reshape(-1)
        lam_g = np.asarray(sol["lam_g"]).reshape(-1)

        success = bool(stats.get("success", False))
        cost = float(sol["f"])

        return x_opt, lam_x, lam_g, (success, cost)

    
    def build_and_solve(self, x0, *decision_vars):
        decision = cs.vertcat(*[v.reshape((-1, 1)) for v in decision_vars])
        nlp = {'x': decision, 'f': self.cost, 'g': cs.vertcat(*self.g)}
        solver = cs.nlpsol(
            "solver",
            "ipopt",
            nlp,
            {
                "ipopt": {
                    "max_iter": 300,
                    "print_level": 0,
                    "tol": 1e-4,
                    "acceptable_tol": 1e-3,
                    "acceptable_iter": 5,
                    "linear_solver": "mumps"
                },
                "print_time": False,
            },
        )
        sol = solver(
            x0=cs.DM(x0),
            lbg=cs.DM(self.lbg),
            ubg=cs.DM(self.ubg),
        )
        success = solver.stats()['success']
        cost = sol['f']
        return np.asarray(sol["x"]).flatten(), (success, cost)