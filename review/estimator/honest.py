"""Proposed variance model for Hold, as a drop-in subclass (the position estimate is unchanged).

Hold sends   var = kf_var + spread^2/n + (ATT_NOISE*h)^2 + (HGT_ERR/h*|d|)^2
which counts ONE attitude error. The fix has TWO independent ones: the error of the attitude used
when the anchor (first keyframe) was made, inherited by every later fix, and the error of the
current frame's attitude. A later keyframe made on the same frame as a fix carries the old
anchor error over almost unchanged (the frame's own attitude error cancels between the fix and
the new G), so a switch adds only a small term, not a whole fix variance.

Model (per axis; var_trace = 2 * per-axis, which is what sim_ground's false-fix test assumes):
  s_anchor^2 = (SIG_TH * h_anchor)^2 + sum over switches of spread^2/(2n) + carried   (per chain)
  s_frame^2  = (SIG_TH * h)^2                                                    (each frame)
  s_bias^2   = 2 * (SIG_BIAS * h)^2 * (1 - exp(-(t - t_anchor) / TAU_BIAS))      (tilt bias change)
  s_yaw^2    = (2 h BETA_UNC sin(|dpsi|/2))^2 / 2   dpsi = yaw change since the anchor
               (uncalibrated camera-mount / FC level error BETA_UNC; see e7b_mount_yaw.py)
  s_local^2  = spread^2 / (2 n) + (HGT_ERR / h * |d|)^2 / 2                     (as in Hold)
SIG_TH is the per-axis, per-frame attitude error the camera sees: FC attitude noise, vibration
the FC cannot see, timing error, camera-mount error. In the sim it measures 0.12-0.14 deg
(e1_variance.py). On hardware set it from the hover's own jitter: SIG_TH ~ std(first difference
of the fix, per axis) / (sqrt(2) * h); Hold can track that online.
SIG_BIAS / TAU_BIAS: the FC tilt-error wander (0 in sim_ground; e7_long_hover.py adds one).
"""
import math

import numpy as np

from hold import Hold


class HonestHold(Hold):
    @staticmethod
    def _yaw(att, t):
        a = att.array()
        return float(np.interp(t, a[:, 0], np.unwrap(a[:, 3])))

    SIG_TH = math.radians(0.13)
    BETA_UNC = math.radians(0.6)   # sim: |ATT_BIAS roll, pitch| = 0.58 deg; hardware: mount tolerance
    SIG_BIAS = 0.0          # rad; FC tilt-bias wander, Gauss-Markov
    TAU_BIAS = 30.0         # s

    def reanchor(self, pos, var_axis=0.0):
        super().reanchor(pos)
        self.chain = None
        self.carried = var_axis           # the EKF's own per-axis variance at the re-anchor

    def process(self, img, t0, att, hgt):
        kf_before = getattr(self, "kf", None)
        e = super().process(img, t0, att, hgt)
        if not hasattr(self, "carried"):
            self.carried = 0.0
        if self.kf is not None and kf_before is None:            # a new chain (init or after lost)
            h = float(hgt.at(e.t))
            if getattr(self, "chain", None) is not None:          # lost without re-anchor: keep what we had
                self.carried = self.chain["last"]
            self.chain = dict(h=h, t=e.t, acc=0.0, last=self.carried, yaw=self._yaw(att, e.t))
        self.old_var = e.var
        if e.var is None:
            return e
        hm = float(hgt.at(e.t))
        kfv = kf_before["var"] if kf_before is not None else 0.0
        local_trace = max(e.var - kfv - (self.ATT_NOISE * hm) ** 2, 0.0)   # spread^2/n + height-scale term
        c = self.chain
        dt = e.t - c["t"]
        s_anchor = (self.SIG_TH * c["h"]) ** 2 + c["acc"] + self.carried
        dpsi = self._yaw(att, e.t) - c["yaw"]
        s_yaw = 0.5 * (2 * hm * self.BETA_UNC * math.sin(min(abs(dpsi), math.pi) / 2)) ** 2
        s_frame = (self.SIG_TH * hm) ** 2
        s_bias = 2 * (self.SIG_BIAS * hm) ** 2 * (1 - math.exp(-max(dt, 0.0) / self.TAU_BIAS))
        var_axis = s_anchor + s_frame + s_bias + s_yaw + local_trace / 2
        c["last"] = var_axis
        if kf_before is not None and self.kf is not kf_before:   # keyframe switch on this frame
            c["acc"] += local_trace / 2       # the hand-over passes on the mean error of this fix's points
        self.var_axis = var_axis
        return e._replace(var=2 * var_axis)                       # trace, like Hold's test assumes
