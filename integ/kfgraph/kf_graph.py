"""Small keyframe pose graph for the VIO position hold (prototype, numpy only, outside hold.py).

Frames (2-D, metres, radians)
- The world frame is the frame of the home keyframe (the gauge: its node is held fixed).
- Each keyframe k has a LOCAL frame: the frame its ground points G were written in. Hold writes G = pos_k + rays, so the
  local frame is labelled by the stored anchor pos_k (2,).
- Node state of keyframe k: anchor a_k (2,) and yaw th_k. A local point x maps to the world as
      world = a_k + R(th_k) (x - pos_k).
  A new node starts at a_k = its anchor in the world, th_k = the frame's yaw in the world.
- One frame matched to keyframe k (hold robust_fit: G ~ C + s R(phi) g) gives the frame's position C in local frame k
  and the turn phi from the frame's rays to local frame k. With the frame's world position p and yaw psi:
      C   = pos_k + R(-th_k) (p - a_k)
      phi = psi - th_k
- An edge between keyframes i and j (one frame matched to both, or a new keyframe made from a fix on another):
      dth = phi_i - phi_j,   dt = (C_i - pos_i) - R(dth) (C_j - pos_j)
  predicted by the node states as dt = R(-th_i) (a_j - a_i), dth = th_j - th_i.
  Each frame's attitude error moves C_i and C_j by the same amount, so it cancels in the edge. The caller gives each
  measurement a covariance WITHOUT the frame's common errors and adds those to the output covariance.

Solver: Gauss-Newton over the free nodes and the current frame, Huber weights on the whitened residual of each block,
and a chi-square gate (3 degrees of freedom) that switches off any block that is still far off after the first
iteration. Priors: a weak prior on each node (keeps the problem regular), a yaw prior on each node and on the frame
(the FC yaw is close to the true yaw over a short hover; YAW_PRIOR is assumed), and a re-anchor prior on the first
node of a chain that started at the FC position.

No I/O. Used by integ/kfgraph/hold_graph.py (option KF_GRAPH).
"""
import math
import time

import numpy as np

CHI2_3_999 = 16.27            # chi-square, 3 degrees of freedom, 99.9 % (table value)
_P = np.array([[0.0, -1.0], [1.0, 0.0]])     # d R(a) / da = R(a) P


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def _rots(a):
    """R(a) for an array of angles: (N, 2, 2)."""
    c, s = np.cos(a), np.sin(a)
    return np.stack([np.stack([c, -s], -1), np.stack([s, c], -1)], -2)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def edge_from_pair(mi, mj, pos_i, pos_j):
    """Relative measurement (z (3,), cov (3, 3)) of node j against node i from one frame matched to both.

    mi, mj: (C (2,), phi, S (3, 3)): the frame in local frame i / j and its covariance (x, y, phi).
    """
    Ci, phi_i, Si = mi
    Cj, phi_j, Sj = mj
    dth = wrap(phi_i - phi_j)
    R = _rot(dth)
    vj = np.asarray(Cj, float) - pos_j
    dt = np.asarray(Ci, float) - pos_i - R @ vj
    Rp = R @ _P @ vj
    Ji = np.zeros((3, 3))
    Ji[:2, :2], Ji[:2, 2], Ji[2, 2] = np.eye(2), -Rp, 1.0
    Jj = np.zeros((3, 3))
    Jj[:2, :2], Jj[:2, 2], Jj[2, 2] = -R, Rp, -1.0
    return np.r_[dt, dth], Ji @ Si @ Ji.T + Jj @ Sj @ Jj.T


class Graph:
    """Nodes: home (fixed) + up to WINDOW others. Edges: one per node pair, the information-weighted mean of its
    per-frame measurements, its covariance shrunk by at most EDGE_NMAX (frames in a row are not independent)."""

    def __init__(self, window=8, huber=2.8, gate=CHI2_3_999, iters=3, edge_nmax=5.0, yaw_prior=math.radians(2.0),
                 weak_prior=10.0):
        self.WINDOW, self.HUBER, self.GATE, self.ITERS = window, huber, gate, iters
        self.EDGE_NMAX, self.YAW_PRIOR, self.WEAK = edge_nmax, yaw_prior, weak_prior
        self.nodes = {}           # id -> dict(pos, a, th, prior (a0, th0, cov3) or None, home, cov3 (last marginal))
        self.edges = {}           # (i, j), i < j -> dict(W (3, 3) info sum, Wz (3,), n, S1 (3, 3) first cov)
        self.home = None
        self.order = []           # non-home node ids, oldest first
        self.stats = dict(edge_frames=0, edge_gated=0, solves=0, blocks_gated=0, dropped=0)
        self.last = None          # info of the last solve

    # ---- nodes ----
    def add_node(self, nid, pos, a, th, prior=None):
        """pos: stored anchor (label of the local frame). a, th: start state. prior: (a0, th0, cov3) or None.
        The first node is home (fixed). Returns [(id, node dict)] of the nodes dropped from the window."""
        home = self.home is None
        self.nodes[nid] = dict(pos=np.asarray(pos, float).copy(), a=np.asarray(a, float).copy(), th=float(th),
                               prior=prior, home=home, cov3=None)
        if home:
            self.home = nid
            self.nodes[nid]["a"], self.nodes[nid]["th"] = np.asarray(pos, float).copy(), 0.0
            return []
        self.order.append(nid)
        out = []
        while len(self.order) > self.WINDOW:
            d = self.order[0]
            out.append((d, self.drop_node(d)))
        return out

    def drop_node(self, nid):
        """Remove a node and its edges. A node that then has no path to home keeps its last solved state as a prior
        (with its last marginal covariance): a cheap stand-in for marginalisation."""
        if nid not in self.nodes or nid == self.home:
            return None
        node = self.nodes.pop(nid)
        self.order = [k for k in self.order if k != nid]
        self.edges = {k: e for k, e in self.edges.items() if nid not in k}
        self.stats["dropped"] += 1
        linked = self._component(self.home)
        for k, n in self.nodes.items():
            if k not in linked and n["prior"] is None:
                c = n["cov3"] if n["cov3"] is not None else np.diag([self.WEAK ** 2] * 2 + [self.YAW_PRIOR ** 2])
                n["prior"] = (n["a"].copy(), n["th"], c)
        return node

    def _component(self, start):
        if start is None:
            return set()
        adj = {}
        for i, j in self.edges:
            adj.setdefault(i, set()).add(j)
            adj.setdefault(j, set()).add(i)
        seen, todo = {start}, [start]
        while todo:
            for m in adj.get(todo.pop(), ()):
                if m not in seen:
                    seen.add(m)
                    todo.append(m)
        return seen

    def linked_to_home(self, nid):
        return nid in self._component(self.home)

    def world(self, nid, C, phi):
        """World position and yaw of a frame measured as (C, phi) in node nid's local frame."""
        n = self.nodes[nid]
        return n["a"] + _rot(n["th"]) @ (np.asarray(C, float) - n["pos"]), n["th"] + phi

    # ---- edges ----
    def add_pair(self, i, mi, j, mj):
        """One frame matched to nodes i and j: add its relative measurement to the (i, j) edge, unless it fails the
        gate against what the edge already holds. Returns True when it was added."""
        if i == j or i not in self.nodes or j not in self.nodes:
            return False
        if i > j:
            i, j, mi, mj = j, i, mj, mi
        z, S = edge_from_pair(mi, mj, self.nodes[i]["pos"], self.nodes[j]["pos"])
        e = self.edges.get((i, j))
        self.stats["edge_frames"] += 1
        if e is not None and e["n"] >= 3:
            zm, Sm = self._edge_mean(e)
            d = z - zm
            d[2] = wrap(d[2])
            if float(d @ np.linalg.solve(S + Sm, d)) > self.GATE:
                e["gated"] += 1
                self.stats["edge_gated"] += 1
                return False
        W = np.linalg.inv(S)
        if e is None:
            self.edges[(i, j)] = dict(W=W, Wz=W @ z, n=1, gated=0, z0=z[2])
        else:
            z = z.copy()
            z[2] = e["z0"] + wrap(z[2] - e["z0"])           # keep the angles of one edge on one branch
            e["W"], e["Wz"], e["n"] = e["W"] + W, e["Wz"] + W @ z, e["n"] + 1
        return True

    def _edge_mean(self, e):
        S = np.linalg.inv(e["W"])
        return S @ e["Wz"], S * max(1.0, e["n"] / self.EDGE_NMAX)

    # ---- solve ----
    def solve(self, meas, psi0=None):
        """meas: list of (nid, C (2,), phi, S (3, 3)) for the current frame. Returns dict(p, psi, cov (2, 2), ...).

        The node states are updated in place (warm start for the next frame). The first measurement starts the frame.
        """
        t_start = time.perf_counter()
        meas = [m for m in meas if m[0] in self.nodes]
        if not meas:
            return None
        free = list(self.order)
        idx = {k: i for i, k in enumerate(free)}
        nf = len(free)
        fr = nf                                  # frame block index
        hi = nf + 1                              # home: a dummy block, dropped from the system
        bi = lambda k: hi if k == self.home else idx[k]
        x_a = np.array([self.nodes[k]["a"] for k in free] + [np.zeros(2), self.nodes[self.home]["a"]]).reshape(-1, 2)
        x_t = np.array([self.nodes[k]["th"] for k in free] + [0.0, self.nodes[self.home]["th"]], float)
        k0, C0, phi0, _ = meas[0]
        p0, ps0 = self.world(k0, C0, phi0)
        x_a[fr], x_t[fr] = p0, ps0 if psi0 is None else psi0
        pos = np.array([self.nodes[k]["pos"] for k in free] + [np.zeros(2), self.nodes[self.home]["pos"]]).reshape(-1, 2)

        # pair blocks: edges (i, j) and frame measurements (node k, frame); the frame plays "j" with a unary model
        E_i, E_j, E_z, E_W, kind = [], [], [], [], []
        for (i, j), e in self.edges.items():
            zm, Sm = self._edge_mean(e)
            E_i.append(bi(i)), E_j.append(bi(j)), E_z.append(zm), E_W.append(np.linalg.inv(Sm)), kind.append(0)
        for k, C, phi, S in meas:
            E_i.append(bi(k)), E_j.append(fr), E_z.append(np.r_[C, phi]), E_W.append(np.linalg.inv(S)), kind.append(1)
        E_i, E_j, kind = np.array(E_i), np.array(E_j), np.array(kind)
        E_z, E_W = np.array(E_z, float), np.array(E_W, float)
        # priors on single blocks: weak position + yaw on every free node and the frame, re-anchor priors
        P_b, P_z, P_W = [], [], []
        wy = 1.0 / self.YAW_PRIOR ** 2
        for k in free:
            n = self.nodes[k]
            P_b.append(idx[k]), P_z.append(np.r_[n["a"], 0.0]), P_W.append(np.diag([self.WEAK ** -2] * 2 + [wy]))
            if n["prior"] is not None:
                a0, t0, c3 = n["prior"]
                P_b.append(idx[k]), P_z.append(np.r_[a0, t0]), P_W.append(np.linalg.inv(c3))
        P_b.append(fr), P_z.append(np.r_[p0, 0.0]), P_W.append(np.diag([self.WEAK ** -2] * 2 + [wy]))
        P_b, P_z, P_W = np.array(P_b), np.array(P_z, float), np.array(P_W, float)

        nb = nf + 2                              # free nodes, frame, home dummy (index nf + 1)
        w_E = np.ones(len(E_i))
        gated = np.zeros(len(E_i), bool)
        it, new = 0, False
        for it in range(self.ITERS + 2):
            r, Ji, Jj = self._pairs(x_a, x_t, pos, E_i, E_j, E_z, kind)
            m2 = np.einsum("ni,nij,nj->n", r, E_W, r)
            if it >= 1:                          # gate after the first step: blocks still far off are switched off
                new = bool(((m2 > self.GATE) & ~gated).any())
                gated |= m2 > self.GATE
                if not new and (it >= self.ITERS or conv):
                    break                        # converged (or out of iterations) with no newly gated block
            m = np.sqrt(np.maximum(m2, 1e-12))
            w_E = np.where(gated, 0.0, np.where(m <= self.HUBER, 1.0, self.HUBER / m))
            H = np.zeros((3 * nb, 3 * nb))
            g = np.zeros(3 * nb)
            WJi = np.einsum("n,nij,njk->nik", w_E, E_W, Ji)
            WJj = np.einsum("n,nij,njk->nik", w_E, E_W, Jj)
            Wr = np.einsum("n,nij,nj->ni", w_E, E_W, r)
            H4 = H.reshape(nb, 3, nb, 3)
            g3 = g.reshape(nb, 3)
            for A, ia in ((Ji, E_i), (Jj, E_j)):
                for WB, ib in ((WJi, E_i), (WJj, E_j)):
                    np.add.at(H4, (ia, slice(None), ib), np.einsum("nji,njk->nik", A, WB))
                np.add.at(g3, ia, np.einsum("nji,nj->ni", A, Wr))
            # priors: residual z - x, Jacobian of the prediction = identity
            pr = P_z - np.c_[x_a[P_b], x_t[P_b]]
            pr[:, 2] = (pr[:, 2] + math.pi) % (2 * math.pi) - math.pi
            for n in range(len(P_b)):
                s = slice(3 * P_b[n], 3 * P_b[n] + 3)
                H[s, s] += P_W[n]
                g[s] += P_W[n] @ pr[n]
            keep = np.r_[0:3 * (nf + 1)]         # drop the home dummy block
            Hk, gk = H[np.ix_(keep, keep)], g[keep]
            dx = np.linalg.solve(Hk, gk)
            dx = dx.reshape(-1, 3)
            x_a[:nf + 1] += dx[:, :2]
            x_t[:nf + 1] += dx[:, 2]
            conv = np.abs(dx[:, :2]).max() < 1e-4 and np.abs(dx[:, 2]).max() < 1e-6
        cov = np.linalg.inv(Hk)
        for k in free:
            n = self.nodes[k]
            n["a"], n["th"] = x_a[idx[k]].copy(), float(x_t[idx[k]])
            n["cov3"] = cov[3 * idx[k]:3 * idx[k] + 3, 3 * idx[k]:3 * idx[k] + 3].copy()
        cf = cov[3 * fr:3 * fr + 3, 3 * fr:3 * fr + 3]
        r, _, _ = self._pairs(x_a, x_t, pos, E_i, E_j, E_z, kind)
        m2 = np.einsum("ni,nij,nj->n", r, E_W, r)
        um = kind == 1
        self.stats["solves"] += 1
        self.stats["blocks_gated"] += int(gated.sum())
        self.last = dict(p=x_a[fr].copy(), psi=float(x_t[fr]), cov=cf[:2, :2].copy(), cov3=cf.copy(), iters=it + 1,
                         ms=1e3 * (time.perf_counter() - t_start), n_nodes=nf + 1, n_edges=int((kind == 0).sum()),
                         n_meas=int(um.sum()), gated_edges=int((gated & ~um).sum()),
                         meas_ids=[m[0] for m in meas], meas_m2=[float(v) for v in m2[um]],
                         meas_gated=[bool(v) for v in gated[um]], meas_w=[float(v) for v in w_E[um]])
        return self.last

    @staticmethod
    def _pairs(x_a, x_t, pos, E_i, E_j, E_z, kind):
        """Residuals r = z - pred (n, 3) and the Jacobians of pred w.r.t. block i and block j (n, 3, 3).

        kind 0 (edge i-j):   pred = [R(-th_i)(a_j - a_i), th_j - th_i]
        kind 1 (node i, frame j = (p, psi)): pred = [pos_i + R(-th_i)(p - a_i), psi - th_i]
        Both have the same Jacobians; kind 1 adds pos_i to the position part.
        """
        ai, aj, ti, tj = x_a[E_i], x_a[E_j], x_t[E_i], x_t[E_j]
        Rm = _rots(-ti)                                   # R(-th_i)
        d = aj - ai
        pt = np.einsum("nij,nj->ni", Rm, d) + np.where(kind[:, None] == 1, pos[E_i], 0.0)
        r = E_z - np.c_[pt, tj - ti]
        r[:, 2] = (r[:, 2] + math.pi) % (2 * math.pi) - math.pi
        n = len(E_i)
        Ji, Jj = np.zeros((n, 3, 3)), np.zeros((n, 3, 3))
        Ji[:, :2, :2] = -Rm
        Ji[:, :2, 2] = -np.einsum("nij,jk,nk->ni", Rm, _P, d)
        Ji[:, 2, 2] = -1.0
        Jj[:, :2, :2] = Rm
        Jj[:, 2, 2] = 1.0
        return r, Ji, Jj
