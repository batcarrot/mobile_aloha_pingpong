


import numpy as np
from pathlib import Path
import pinocchio as pin
from pinocchio import casadi as cpin
import casadi as cs
import re
import math

from physics_utils.common import *
from physics_utils import OCPSolver


asset_path = Path(__file__).parent / 'aloha_paddle.usd'
asset_path = asset_path.as_posix()

urdf_path = '/home/trossen-ai/mobile_aloha_pingpong/assets/aloha.urdf'

class AlohaArmCasadi:
    def __init__(self, free_joint_expr='*', ee_frame_name='fr_link6', paddle_orientation='down'):
        
        
        self.model = pin.buildModelFromUrdf(urdf_path)
        
        all_joints = [x for x in self.model.names]
        print(all_joints)
        free_joints = [x for x in all_joints if re.match(free_joint_expr, x)]
        print(free_joints)
        lock_joints = list(set(all_joints) - set(free_joints))
        lock_joints.remove('universe')
        lock_joints_id = list(set([self.model.getJointId(x) for x in lock_joints]))
        self.model = pin.buildReducedModel(self.model, lock_joints_id, np.zeros(self.model.nq))
        print(self.model)
        self.data = self.model.createData()
        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()
        
        self.ee_frame_name = ee_frame_name
        self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)
        
        self.ee_local_offset = np.array([0.25, 0., 0.])
        self.normal_dir = cs.DM([[0.], [0.], [1.]])
        self.ee_local_offset_dm = cs.DM(self.ee_local_offset)
        
        self.q_lower = np.asarray(self.model.lowerPositionLimit).flatten()
        self.q_upper = np.asarray(self.model.upperPositionLimit).flatten()
        
        nq = self.model.nq
        q = cs.SX.sym("q", nq, 1)

        ee, eeori = self.fk_expr(q)
        J = self.jac_expr(q)
        fwd = eeori - ee

        self._fk_fun = cs.Function("fk_fun", [q], [ee, eeori, fwd])
        self._jac_fun = cs.Function("jac_fun", [q], [J])
    
        self.reset_solver()
    
    def reset_solver(self):
        self.active_solver = None
        self.x0 = None
        self.lam_x = None
        self.lam_g = None
        
    def fk_expr(self, q: cs.MX):
        q_vec = cs.reshape(q, (self.model.nq, 1))
        cpin.forwardKinematics(self.cmodel, self.cdata, q_vec)
        cpin.updateFramePlacements(self.cmodel, self.cdata)
        pos = self.cdata.oMf[self.ee_frame_id].translation
        pos += self.cdata.oMf[self.ee_frame_id].rotation @ self.ee_local_offset_dm
        forward_pos = pos + cs.mtimes(
            self.cdata.oMf[self.ee_frame_id].rotation,
            self.normal_dir
        )
        return pos, forward_pos
    
    def jac_numeric(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(-1)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        J6 = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            self.ee_frame_id,
            pin.ReferenceFrame.WORLD
        )  # 6 x nq

        return np.asarray(J6[:3, :], dtype=float)
    
    def jac_expr(self, q: cs.MX):
        q_vec = cs.reshape(q, (self.model.nq, 1))
        cpin.forwardKinematics(self.cmodel, self.cdata, q_vec)
        cpin.updateFramePlacements(self.cmodel, self.cdata)
        J = cpin.computeFrameJacobian(
            self.cmodel,
            self.cdata,
            q_vec,
            self.ee_frame_id,
            pin.ReferenceFrame.WORLD
        )
        J_pos = J[0:3, :]
        return J_pos
    
    def fk_numeric(self, q: np.ndarray) -> np.ndarray:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pos = self.data.oMf[self.ee_frame_id].translation
        
        pos += self.data.oMf[self.ee_frame_id].rotation @ self.ee_local_offset
        
        return np.asarray(pos).flatten()
    def solve_ocp_bezier_qp(
        self,
        p_des: np.ndarray,
        v_des: np.ndarray,
        o_des: np.ndarray,
        q0: np.ndarray,
        qd0: np.ndarray,
        t_s: float,
        t_f: float,
        horizon: int = 8,
        wa: float = 1e-2,
        wr: float = 1e-1,
        wp: float = 1e3,
        wv: float = 1e2,
        wn: float = 1e2,
        sqp_iters: int = 4,
        trust_region: float = 0.2,
    ):
        """
        SQP loop where each iteration solves a convex QP with OSQP via CasADi's low-level conic interface.
        This matches CasADi docs section 4.6.2 (low-level interface). :contentReference[oaicite:2]{index=2}
        """

        nq = int(self.model.nq)
        n = int(horizon)
        if n < 2:
            raise ValueError("horizon must be >= 2")
        if t_f <= t_s:
            raise ValueError("t_f must be larger than t_s")
        T = float(t_f - t_s)

        # --- Bernstein basis (numeric) ---
        def bernstein(deg: int, tau: float) -> np.ndarray:
            return np.array(
                [math.comb(deg, i) * (tau**i) * ((1.0 - tau) ** (deg - i)) for i in range(deg + 1)],
                dtype=float,
            )

        taus = np.linspace(0.0, 1.0, n + 1)
        Bn  = np.stack([bernstein(n,   tau) for tau in taus], axis=1)  # (n+1, n+1)
        Bn1 = np.stack([bernstein(n-1, tau) for tau in taus], axis=1)  # (n,   n+1)
        Bn2 = np.stack([bernstein(n-2, tau) for tau in taus], axis=1)  # (n-1, n+1)

        dim_z = nq * (n + 1)

        # vec(q_c) uses Fortran order: columns stacked.
        def col_selector(j: int) -> np.ndarray:
            e = np.zeros((n + 1,), dtype=float)
            e[j] = 1.0
            return np.kron(e, np.eye(nq))  # (nq, dim_z)

        # Linear maps: q_k = Mq[k] z, qd_k = Mqd[k] z, qdd_k = Mqdd[k] z
        Mq, Mqd, Mqdd = [], [], []
        for k in range(n + 1):
            bk = Bn[:, k]
            Mq.append(np.kron(bk, np.eye(nq)))  # (nq, dim_z)

            # qd_k
            Mk = np.zeros((nq, dim_z), dtype=float)
            if n - 1 > 0:
                b1 = Bn1[:, k]  # length n
                for i in range(n):
                    Mk += b1[i] * (col_selector(i + 1) - col_selector(i))
                Mk *= (n / T)
            Mqd.append(Mk)

            # qdd_k
            Mk2 = np.zeros((nq, dim_z), dtype=float)
            if n - 2 > 0:
                b2 = Bn2[:, k]  # length n-1
                for i in range(n - 1):
                    Mk2 += b2[i] * (col_selector(i + 2) - 2 * col_selector(i + 1) + col_selector(i))
                Mk2 *= (n * (n - 1) / (T * T))
            Mqdd.append(Mk2)

        # --- CasADi terminal linearization functions ---
        q_sym = cs.SX.sym("qT", nq, 1)
        ee_sym, eeori_sym = self.fk_expr(q_sym)
        J_sym = self.jac_expr(q_sym)  # expect 3 x nq
        fwd_sym = eeori_sym - ee_sym  # your proxy vector

        Jee_sym = cs.jacobian(ee_sym, q_sym)       # 3 x nq
        Jfwd_sym = cs.jacobian(fwd_sym, q_sym)     # 3 x nq
        fk_lin_fun = cs.Function("fk_lin", [q_sym], [ee_sym, Jee_sym, J_sym, fwd_sym, Jfwd_sym])

        # --- Warm start control points (constant-vel extrapolation) ---
        q_c_init = np.zeros((nq, n + 1), dtype=float)
        q_c_init[:, 0] = q0
        q_c_init[:, 1] = q0 + qd0 * (T / n)
        for i in range(2, n + 1):
            q_c_init[:, i] = q_c_init[:, i - 1] + (q_c_init[:, i - 1] - q_c_init[:, i - 2])
        z = q_c_init.reshape((-1,), order="F")

        # --- Build constant linear constraints pieces once ---
        ql = np.array(self.q_lower, dtype=float).reshape((nq,))
        qu = np.array(self.q_upper, dtype=float).reshape((nq,))

        A_blocks, l_blocks, u_blocks = [], [], []

        # Joint limits at all nodes: ql <= Mq[k] z <= qu
        for k in range(n + 1):
            A_blocks.append(Mq[k])
            l_blocks.append(ql)
            u_blocks.append(qu)

        # Initial constraints: Mq[0] z == q0, Mqd[0] z == qd0
        A_blocks.append(Mq[0]);  l_blocks.append(np.array(q0,  dtype=float)); u_blocks.append(np.array(q0,  dtype=float))
        A_blocks.append(Mqd[0]); l_blocks.append(np.array(qd0, dtype=float)); u_blocks.append(np.array(qd0, dtype=float))

        A_base = np.vstack(A_blocks)
        l_base = np.concatenate(l_blocks).astype(float)
        u_base = np.concatenate(u_blocks).astype(float)

        # Trust region constraints will be appended each SQP iter:  z_prev - tr <= z <= z_prev + tr
        # That is: I z in [z_prev-tr, z_prev+tr]
        I = np.eye(dim_z)

        # --- Create OSQP conic solver ONCE with sparsity patterns (doc low-level interface) ---
        # conic expects qp={'h': H.sparsity(), 'a': A.sparsity()} when constructing. :contentReference[oaicite:3]{index=3}
        # We'll include the trust region rows in A's sparsity by building with full A pattern once.
        A_full = np.vstack([A_base, I])
        qp_struct = {
            "h": cs.DM(np.eye(dim_z)).sparsity(),  # placeholder sparsity (we'll pass numeric H each call)
            "a": cs.DM(A_full).sparsity(),
        }
        S = cs.conic("S", "osqp", qp_struct, {
            "warm_start_primal": True,
            "warm_start_dual": True,
            "error_on_fail": False,
            "osqp": {   
                "verbose": True,
                "eps_abs": 1e-3,
                "eps_rel": 1e-3,
                "max_iter": 2000,
                # other OSQP settings if you want:
                # "polish": False,
                # "alpha": 1.6,
            },
        })

        # --- SQP iterations ---
        p = p_des.reshape((3, 1))
        v = v_des.reshape((3, 1))
        o = o_des.reshape((3, 1))

        for _ in range(int(sqp_iters)):
            z_prev = z.copy()

            # terminal state under current z
            qT = (Mq[n] @ z).reshape((nq, 1))
            qdT = (Mqd[n] @ z).reshape((nq, 1))

            ee0, Jee0, J0, fwd0, Jfwd0 = fk_lin_fun(cs.DM(qT))
            ee0   = np.array(ee0).reshape((3, 1))
            Jee0  = np.array(Jee0)     # 3 x nq
            J0    = np.array(J0)       # 3 x nq
            fwd0  = np.array(fwd0).reshape((3, 1))
            Jfwd0 = np.array(Jfwd0)    # 3 x nq

            MqT = Mq[n]      # nq x dim_z
            MqdT = Mqd[n]    # nq x dim_z

            # Linearized terminal mappings:
            # ee(qT) ≈ ee0 + Jee0 (MqT z - qT0) = (Jee0 MqT) z + (ee0 - Jee0 qT0)
            Aee = Jee0 @ MqT
            bee = ee0 - Jee0 @ qT

            # velocity (cheap approximation): v ≈ J(qT0) * qdT = (J0 MqdT) z
            Av = J0 @ MqdT
            bv = np.zeros((3, 1))

            # forward vector fwd(qT) ≈ (Jfwd0 MqT) z + (fwd0 - Jfwd0 qT0)
            Af = Jfwd0 @ MqT
            bf = fwd0 - Jfwd0 @ qT

            # Build convex QP objective 0.5 z^T H z + g^T z
            H = np.zeros((dim_z, dim_z), dtype=float)
            g = np.zeros((dim_z,), dtype=float)

            # smoothness costs
            for k in range(n + 1):
                H += 2.0 * wr * (Mqd[k].T @ Mqd[k])
                if k < n:
                    H += 2.0 * wa * (Mqdd[k].T @ Mqdd[k])

            # terminal penalties
            r0 = (bee - p)  # 3x1
            H += 2.0 * wp * (Aee.T @ Aee)
            g += (2.0 * wp * (Aee.T @ r0)).reshape(-1)

            r1 = (bv - v)
            H += 2.0 * wv * (Av.T @ Av)
            g += (2.0 * wv * (Av.T @ r1)).reshape(-1)

            r2 = (bf - o)
            H += 2.0 * wn * (Af.T @ Af)
            g += (2.0 * wn * (Af.T @ r2)).reshape(-1)

            # tiny regularization for numerical stability
            H += 1e-6 * np.eye(dim_z)

            # Constraints: base + trust region
            A = np.vstack([A_base, I])
            l = np.concatenate([l_base, z_prev - trust_region]).astype(float)
            u = np.concatenate([u_base, z_prev + trust_region]).astype(float)

            # Solve via conic: pass numeric h,g,a,lba,uba plus optional x0. :contentReference[oaicite:4]{index=4}
            sol = S(
                h=cs.DM(H),
                g=cs.DM(g),
                a=cs.DM(A),
                lba=cs.DM(l),
                uba=cs.DM(u),
                x0=cs.DM(z_prev),
            )
            z = np.array(sol["x"]).reshape((-1,))

        # decode trajectory at nodes
        q_mat = np.zeros((n + 1, nq))
        qd_mat = np.zeros((n + 1, nq))
        qdd_mat = np.zeros((n, nq))
        for k in range(n + 1):
            q_mat[k] = (Mq[k] @ z)
            qd_mat[k] = (Mqd[k] @ z)
            if k < n:
                qdd_mat[k] = (Mqdd[k] @ z)

        # compute “true” terminal errors (nonlinear) for a success flag
        if not hasattr(self, "_fk_fun"):
            nq = self.model.nq
            q_eval = cs.SX.sym("q_eval", nq, 1)
            ee_e, eeori_e = self.fk_expr(q_eval)
            J_e = self.jac_expr(q_eval)
            fwd_e = eeori_e - ee_e
            self._fk_fun = cs.Function("fk_fun", [q_eval], [ee_e, eeori_e, fwd_e])
            self._jac_fun = cs.Function("jac_fun", [q_eval], [J_e])

        # numeric evaluation
        qT_dm = cs.DM(q_mat[-1].reshape((nq, 1)))
        qdT_dm = cs.DM(qd_mat[-1].reshape((nq, 1)))

        eeT_dm, eeoriT_dm, fwdT_dm = self._fk_fun(qT_dm)
        JT_dm = self._jac_fun(qT_dm)

        eeT = np.array(eeT_dm).reshape((3,))
        vT = np.array(JT_dm @ qdT_dm).reshape((3,))
        fwdT = np.array(fwdT_dm).reshape((3,))

        pos_err = float(np.linalg.norm(eeT - p_des.reshape((3,))))
        vel_err = float(np.linalg.norm(vT - v_des.reshape((3,))))
        fwd_err = float(np.linalg.norm(fwdT - o_des.reshape((3,))))

        success = (pos_err < 5e-3) and (vel_err < 5e-2)

        return q_mat, qd_mat, qdd_mat, success
    def solve_ocp_bezier(self,
                         p_des: np.ndarray,
                         v_des: np.ndarray,
                         o_des: np.ndarray,
                         q0: np.ndarray,
                         qd0: np.ndarray,
                         t_s: float,
                         t_f: float,
                         horizon: int = 8,
                         wa: float = 1e-6,
                         wr: float = 1e-4,
                         eps_p: float = 1e-3,
                         eps_v: float = 1e-5,
                         eps_n: float = 1e-5,):
        nq = self.model.nq
        ocp_solver = OCPSolver()

        if horizon < 2:
            raise ValueError("horizon must be >= 2 for Bezier trajectory optimization")
        if t_f <= t_s:
            raise ValueError("t_f must be larger than t_s")

        T = t_f - t_s
        n = horizon
        q_c = cs.SX.sym('q_c', nq, n + 1)

        def bernstein_basis(deg: int, tau: float) -> cs.DM:
            if deg < 0:
                return cs.DM.zeros(0, 1)
            vals = [
                math.comb(deg, i) * (tau ** i) * ((1.0 - tau) ** (deg - i))
                for i in range(deg + 1)
            ]
            return cs.DM(vals).reshape((deg + 1, 1))

        taus = np.linspace(0.0, 1.0, n + 1)
        b_n = [bernstein_basis(n, tau) for tau in taus]
        b_n1 = [bernstein_basis(n - 1, tau) for tau in taus]
        b_n2 = [bernstein_basis(n - 2, tau) for tau in taus]

        dP = n * (q_c[:, 1:] - q_c[:, :-1])
        ddP = (n - 1) * (dP[:, 1:] - dP[:, :-1])

        q_nodes = []
        qd_nodes = []
        qdd_nodes = []
        for k in range(n + 1):
            qk = q_c @ b_n[k]
            qdk = (dP @ b_n1[k]) / T
            qddk = (ddP @ b_n2[k]) / (T ** 2)
            q_nodes.append(qk)
            qd_nodes.append(qdk)
            qdd_nodes.append(qddk)

        for k in range(n):
            ocp_solver.add_cost(wa * cs.sumsqr(qdd_nodes[k]))
            ocp_solver.add_cost(wr * cs.sumsqr(qd_nodes[k]))

            ocp_solver.add_geq_constraint(q_nodes[k], self.q_lower)
            ocp_solver.add_leq_constraint(q_nodes[k], self.q_upper)

        ocp_solver.add_geq_constraint(q_nodes[n], self.q_lower)
        ocp_solver.add_leq_constraint(q_nodes[n], self.q_upper)
        ocp_solver.add_cost(wr * cs.sumsqr(qd_nodes[n]))

        ee, ee_ori = self.fk_expr(q_nodes[n])
        jac_f = self.jac_expr(q_nodes[n])
        ocp_solver.add_leq_constraint(cs.sumsqr(ee - cs.DM(p_des)), eps_p)
        ocp_solver.add_leq_constraint(cs.sumsqr(jac_f @ qd_nodes[n] - cs.DM(v_des)), eps_v)
        ocp_solver.add_leq_constraint(cs.sumsqr(ee_ori - ee - cs.DM(o_des)), eps_n)

        ocp_solver.add_eq_constraint(q_nodes[0], cs.DM(q0))
        ocp_solver.add_eq_constraint(qd_nodes[0], cs.DM(qd0))

        # Warm start using a constant-velocity line that satisfies q(0)=q0 and qd(0)=qd0.
        q_c_init = np.zeros((nq, n + 1))
        q_c_init[:, 0] = q0
        q_c_init[:, 1] = q0 + qd0 * (T / n)
        for i in range(2, n + 1):
            q_c_init[:, i] = q_c_init[:, i - 1] + (q_c_init[:, i - 1] - q_c_init[:, i - 2])
        x0 = q_c_init.reshape(-1, order="F")

        x_opt, (success, cost) = ocp_solver.solve(x0, q_c)

        q_c_opt = x_opt.reshape((nq, n + 1), order="F")
        dP_opt = n * (q_c_opt[:, 1:] - q_c_opt[:, :-1])
        ddP_opt = (n - 1) * (dP_opt[:, 1:] - dP_opt[:, :-1])

        q_mat = np.zeros((n + 1, nq))
        qd_mat = np.zeros((n + 1, nq))
        qdd_mat = np.zeros((n, nq))
        for k, tau in enumerate(taus):
            b = np.array([
                math.comb(n, i) * (tau ** i) * ((1.0 - tau) ** (n - i))
                for i in range(n + 1)
            ])
            b1 = np.array([
                math.comb(n - 1, i) * (tau ** i) * ((1.0 - tau) ** (n - 1 - i))
                for i in range(n)
            ])
            b2 = np.array([
                math.comb(n - 2, i) * (tau ** i) * ((1.0 - tau) ** (n - 2 - i))
                for i in range(n - 1)
            ])

            q_mat[k] = q_c_opt @ b
            qd_mat[k] = (dP_opt @ b1) / T
            if k < n:
                qdd_mat[k] = (ddP_opt @ b2) / (T ** 2)

        return q_mat, qd_mat, qdd_mat, success

    def solve_ocp(self, 
                  p_des: np.ndarray,
                  v_des: np.ndarray,
                  o_des: np.ndarray,
                  q0: np.ndarray,
                  qd0: np.ndarray,
                  t: float,
                  t_f: float,
                  horizon: int = 30,
                  dt: float = 0.05,
                  wa: float = 1e-2,
                  wv: float = 1e-1,
                  eps_p: float = 1e-3,
                  eps_v: float = 1e-5,
                  eps_n: float = 1e-5,
                  x0=None):
        
        if self.active_solver is None:
            self.active_solver, meta = self.build_ocp_solver(
                horizon=horizon,
                dt=dt,
                wa=wa,
                wv=wv,
                eps_p=eps_p,
                eps_v=eps_v,
                eps_n=eps_n,
            )
            self.ocp_meta = meta
            H = meta["H"]
            nq = meta["nq"]
            self.x0 = np.concatenate([
                np.tile(q0, H + 1),
                np.tile(qd0, H + 1),
                np.zeros(nq * H)  # qdd initial guess
            ])
            self.lam_x = None
            self.lam_g = None
        
        q_sol, qd_sol, qdd_sol, success, cost, x_opt, lam_x, lam_g, k_now = self.solve_ocp_step(
            ocp=self.active_solver,
            meta=self.ocp_meta,
            p_des=p_des,
            v_des=v_des,
            o_des=o_des,
            q_now=q0,
            qd_now=qd0,
            t=t,
            t_f=t_f,
            dt=dt,
            x0=self.x0,
            lam_x=self.lam_x,
            lam_g=self.lam_g,
        )
        
        if success:
            self.x0 = x_opt
            self.lam_x = lam_x
            self.lam_g = lam_g
        
        return q_sol, qd_sol, qdd_sol, cost, success, k_now
            
            
        
    def build_ocp_solver(self, horizon=30, dt=0.05, wa=1e-2, wv=1e-1,
                     eps_p=1e-3, eps_v=1e-5, eps_n=1e-5):
        nq = self.model.nq
        H = horizon

        # decision vars
        q   = cs.SX.sym("q",   nq, H+1)
        qd  = cs.SX.sym("qd",  nq, H+1)
        qdd = cs.SX.sym("qdd", nq, H)

        x = cs.vertcat(cs.vec(q), cs.vec(qd), cs.vec(qdd))

        # parameters
        p_des = cs.SX.sym("p_des", 3)
        v_des = cs.SX.sym("v_des", 3)
        n_des = cs.SX.sym("n_des", 3)
        q_now = cs.SX.sym("q_now", nq)
        qd_now = cs.SX.sym("qd_now", nq)
        mask = cs.SX.sym("mask", H)        # 0/1
        alpha = cs.SX.sym("alpha", H+1)    # one-hot

        p = cs.vertcat(p_des, v_des, n_des, q_now, qd_now, mask, alpha)

        ocp = OCPSolver()

        # dynamics + masked running cost (structure fixed!)
        for k in range(H):
            mk = mask[k]
            ocp.add_cost(mk * wa * cs.sumsqr(qdd[:, k]))
            ocp.add_cost(mk * wv * cs.sumsqr(qd[:, k]))

            ocp.add_eq_constraint(q[:, k+1],  q[:, k]  + qd[:, k] * dt)
            ocp.add_eq_constraint(qd[:, k+1], qd[:, k] + qdd[:, k] * dt)

        ocp.add_cost(wv * cs.sumsqr(qd[:, H]))

        # terminal constraints at strike (always index H)
        ee, ee_ori = self.fk_expr(q[:, H])
        Jpos = self.jac_expr(q[:, H])          # your J_pos (3 x nq)

        ocp.add_leq_constraint(cs.sumsqr(ee - p_des), eps_p)
        ocp.add_leq_constraint(cs.sumsqr(Jpos @ qd[:, H] - v_des), eps_v)
        # ocp.add_leq_constraint(cs.sumsqr(cs.cross(n_des, ee_ori - ee)), eps_n)
        ocp.add_leq_constraint(cs.sumsqr(ee_ori - ee - n_des), eps_n)
        

        # anchor "now" at knot selected by alpha (one-hot)
        ocp.add_eq_constraint(q @ cs.reshape(alpha, (H+1, 1)), cs.reshape(q_now, (nq, 1)))
        ocp.add_eq_constraint(qd @ cs.reshape(alpha, (H+1, 1)), cs.reshape(qd_now, (nq, 1)))

        # (recommended) bounds via lbx/ubx for q across all knots
        # Build lbx/ubx for x = [vec(q), vec(qd), vec(qdd)]
        q_lower = np.asarray(self.q_lower).reshape(-1)
        q_upper = np.asarray(self.q_upper).reshape(-1)

        lbx = -np.inf * np.ones(x.shape[0], dtype=float)
        ubx = +np.inf * np.ones(x.shape[0], dtype=float)

        q_size = nq * (H+1)
        # q block bounds (repeat per knot)
        lbx[:q_size] = np.tile(q_lower, H+1)
        ubx[:q_size] = np.tile(q_upper, H+1)

        ocp.set_x_bounds(lbx, ubx)

        # compile once
        ocp.finalize(x=x, p=p)

        # store shapes/handles you’ll need for unpacking
        return ocp, {"nq": nq, "H": H, "q": q, "qd": qd, "qdd": qdd}
        
    def solve_ocp_step(self, 
                       ocp: OCPSolver, 
                       meta: dict, 
                       p_des: np.ndarray, 
                       v_des: np.ndarray, 
                       o_des: np.ndarray,
                       q_now: np.ndarray, 
                       qd_now: np.ndarray, 
                       t: float, 
                       t_f: float, 
                       dt: float,
                       x0: np.ndarray, 
                       lam_x=None, lam_g=None):
        nq, H = meta["nq"], meta["H"]

        t0 = t_f - H * dt
        k_now = int(np.clip((t - t0) / dt, 0, H))  # allow k_now==H if at strike

        # mask for running controls qdd[:,k] where k in [0..H-1]
        mask = np.zeros(H)
        mask[max(0, min(k_now, H)):] = 1.0  # from now onward

        # one-hot alpha for knots [0..H]
        alpha = np.zeros(H+1)
        alpha[min(k_now, H)] = 1.0

        # pack parameter vector p = [p_des(3), v_des(3), q_now(nq), qd_now(nq), mask(H), alpha(H+1)]
        p = np.concatenate([
            np.asarray(p_des).reshape(3),
            np.asarray(v_des).reshape(3),
            np.asarray(o_des).reshape(3),
            np.asarray(q_now).reshape(nq),
            np.asarray(qd_now).reshape(nq),
            mask,
            alpha
        ])

        x_opt, lam_x, lam_g, (success, cost) = ocp.solve(x0=x0, p=p, lam_x0=lam_x, lam_g0=lam_g)

        # unpack like you already do
        q_size = nq * (H+1)
        qd_size = nq * (H+1)

        q_mat  = x_opt[:q_size].reshape((nq, H+1), order="F").T
        qd_mat = x_opt[q_size:q_size+qd_size].reshape((nq, H+1), order="F").T
        qdd_mat = x_opt[q_size+qd_size:].reshape((nq, H), order="F").T

        return q_mat, qd_mat, qdd_mat, success, cost, x_opt, lam_x, lam_g, k_now

    def inverse_dynamics(self, q, qd, qdd):
        return pin.rnea(self.model, self.data, q, qd, qdd)

    def get_control(self, q, qd, qdd, q_obs, qd_obs, kp=1000, kd=100):
        e = q_obs - q
        ed = qd_obs - qd
        return pin.rnea(self.model, self.data, q, qd, qdd - kp * e - kd * ed)

