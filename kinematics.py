"""
Planar (front-view) double-wishbone suspension kinematics.

Kinematics, not dynamics: we prescribe the wheel's vertical travel and solve the
linkage geometry. No forces, no mass, no time integration. The suspension is a
four-bar (chassis - lower arm - upright - upper arm - chassis) with 1 DOF. An
optional pushrod + rocker + damper group rides on top to give the motion ratio.

Front view (looking along the car's +x axis), models the RIGHT wheel:
    y = lateral, outboard positive ;  z = vertical, up positive ;  x = 0.

Sign conventions (right wheel):
    camber > 0  -> top of wheel leans OUTBOARD (+y)
    travel > 0  -> wheel moves UP (bump / jounce)
    scrub  > 0  -> contact patch moves OUTBOARD (+y)
"""
import numpy as np


def _circle_circle(c0, r0, c1, r1):
    """Intersections of two circles in 2D. Returns (p_plus, p_minus) or None."""
    d = np.linalg.norm(c1 - c0)
    if d < 1e-12 or d > r0 + r1 or d < abs(r0 - r1):
        return None
    a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
    h = np.sqrt(max(r0 ** 2 - a ** 2, 0.0))
    u = (c1 - c0) / d
    mid = c0 + a * u
    perp = np.array([-u[1], u[0]])
    return mid + h * perp, mid - h * perp


def _rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def _tilt(vec):
    """Tilt of an upright vector from vertical (deg). + means top leans outboard."""
    return np.degrees(np.arctan2(vec[0], vec[1]))   # atan2(dy, dz)


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _line_intersection(p, r, q, s):
    """Intersection of line (p + t r) and line (q + u s). None if ~parallel."""
    denom = _cross2(r, s)
    if abs(denom) < 1e-12:
        return None
    t = _cross2(q - p, s) / denom
    return p + t * r


def _roll_center(lca_in, lbj, uca_in, ubj, contact):
    """Front-view roll center: intersect the contact-patch->instant-center line
    with the car centerline (y = 0). Returns ([y=0, z], instant_center)."""
    ic = _line_intersection(lca_in, lbj - lca_in, uca_in, ubj - uca_in)
    direction = (ic - contact) if ic is not None else (lbj - lca_in)
    if abs(direction[0]) < 1e-12:
        return np.array([0.0, np.nan]), ic
    u = -contact[0] / direction[0]
    return np.array([0.0, contact[1] + u * direction[1]]), ic


class DoubleWishbone2D:
    """Front-view double-wishbone defined by hardpoints (each [y, z], metres).

    Optional actuation (pass all five to get the motion ratio):
        pushrod_outboard : pushrod pickup on the LOWER arm (rides with it)
        rocker_pivot     : rocker pivot on the chassis (fixed)
        rocker_pushrod   : pushrod pickup on the rocker (static reference)
        rocker_damper    : damper pickup on the rocker (static reference)
        damper_inboard   : damper mount on the chassis (fixed)
    """

    def __init__(self, lca_inboard, uca_inboard, lower_ball_joint, upper_ball_joint,
                 contact_patch, static_camber=0.0, camber_sign=+1.0,
                 pushrod_outboard=None, rocker_pivot=None, rocker_pushrod=None,
                 rocker_damper=None, damper_inboard=None):
        self.lca_in = np.asarray(lca_inboard, float)
        self.uca_in = np.asarray(uca_inboard, float)
        self.lbj0 = np.asarray(lower_ball_joint, float)
        self.ubj0 = np.asarray(upper_ball_joint, float)
        self.cp0 = np.asarray(contact_patch, float)
        self.static_camber = static_camber
        self.camber_sign = camber_sign

        self.lca_len = np.linalg.norm(self.lbj0 - self.lca_in)
        self.uca_len = np.linalg.norm(self.ubj0 - self.uca_in)
        self.upright_len = np.linalg.norm(self.ubj0 - self.lbj0)

        u0 = self.ubj0 - self.lbj0
        self._ang0 = np.arctan2(u0[1], u0[0])
        self._tilt0 = _tilt(u0)

        # Optional pushrod + rocker + damper actuation group
        self.has_rocker = all(p is not None for p in
                              (pushrod_outboard, rocker_pivot, rocker_pushrod,
                               rocker_damper, damper_inboard))
        if self.has_rocker:
            self.pr_out0 = np.asarray(pushrod_outboard, float)
            self.rk_piv = np.asarray(rocker_pivot, float)
            self.rk_push0 = np.asarray(rocker_pushrod, float)
            self.rk_damp0 = np.asarray(rocker_damper, float)
            self.dmp_in = np.asarray(damper_inboard, float)
            self.pushrod_len = np.linalg.norm(self.pr_out0 - self.rk_push0)
            self.rk_push_r = np.linalg.norm(self.rk_push0 - self.rk_piv)
            self._rk_ang0 = np.arctan2(*(self.rk_push0 - self.rk_piv)[::-1])
            self.damper_len0 = np.linalg.norm(self.rk_damp0 - self.dmp_in)

    def _solve_rocker(self, theta, prev_rk=None):
        # Pushrod outboard pickup rides with the lower arm (rotates by theta)
        pr_out = self.lca_in + _rot(theta) @ (self.pr_out0 - self.lca_in)
        # Rocker pushrod point: on rocker circle AND pushrod-length from pr_out
        sols = _circle_circle(self.rk_piv, self.rk_push_r, pr_out, self.pushrod_len)
        if sols is None:
            raise ValueError(f"Pushrod/rocker cannot close at theta={theta:.4f} rad.")
        ref = self.rk_push0 if prev_rk is None else prev_rk
        rk_push = min(sols, key=lambda p: np.linalg.norm(p - ref))
        # Rocker rotation -> damper point -> damper length
        alpha = np.arctan2(*(rk_push - self.rk_piv)[::-1]) - self._rk_ang0
        rk_damp = self.rk_piv + _rot(alpha) @ (self.rk_damp0 - self.rk_piv)
        damper_len = float(np.linalg.norm(rk_damp - self.dmp_in))
        return {"pr_out": pr_out, "rk_push": rk_push, "rk_damp": rk_damp,
                "damper_len": damper_len}

    def solve(self, theta, prev_ubj=None, prev_rk=None):
        """Forward kinematics for a lower-arm rotation `theta` (rad)."""
        lbj = self.lca_in + _rot(theta) @ (self.lbj0 - self.lca_in)
        sols = _circle_circle(self.uca_in, self.uca_len, lbj, self.upright_len)
        if sols is None:
            raise ValueError(f"Linkage cannot close at theta={theta:.4f} rad; "
                             "reduce the travel range.")
        ref = self.ubj0 if prev_ubj is None else prev_ubj
        ubj = min(sols, key=lambda p: np.linalg.norm(p - ref))

        u = ubj - lbj
        phi = np.arctan2(u[1], u[0]) - self._ang0
        cp = lbj + _rot(phi) @ (self.cp0 - self.lbj0)
        camber = self.static_camber + self.camber_sign * (_tilt(u) - self._tilt0)
        rc, ic = _roll_center(self.lca_in, lbj, self.uca_in, ubj, cp)

        result = {"lbj": lbj, "ubj": ubj, "cp": cp, "phi": phi, "camber": camber,
                  "rc": rc, "rc_height": float(rc[1]), "ic": ic}
        if self.has_rocker:
            result.update(self._solve_rocker(theta, prev_rk=prev_rk))
        return result

    def _cp_of(self, theta):
        """Contact-patch position from the wishbone only (no rocker), for bracketing."""
        lbj = self.lca_in + _rot(theta) @ (self.lbj0 - self.lca_in)
        sols = _circle_circle(self.uca_in, self.uca_len, lbj, self.upright_len)
        if sols is None:
            raise ValueError(f"Linkage cannot close at theta={theta:.4f} rad.")
        ubj = min(sols, key=lambda p: np.linalg.norm(p - self.ubj0))
        phi = np.arctan2(*(ubj - lbj)[::-1]) - self._ang0
        return lbj + _rot(phi) @ (self.cp0 - self.lbj0)

    def _travel_of(self, theta):
        return self._cp_of(theta)[1] - self.cp0[1]

    def _theta_for_travel(self, target, lo=-0.7, hi=0.7, iters=64):
        if self._travel_of(hi) < self._travel_of(lo):
            lo, hi = hi, lo
        for _ in range(iters):
            mid = 0.5 * (lo + hi)
            if self._travel_of(mid) < target:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def sweep(self, travel=0.030, n=41):
        """Sweep -travel (rebound) -> +travel (bump). Returns arrays of outputs.

        If the rocker is defined, also returns `damper_len` and `motion_ratio`
        (= |d damper length / d wheel travel|, the damper/wheel convention)."""
        thetas = np.linspace(self._theta_for_travel(-travel),
                             self._theta_for_travel(+travel), n)
        out = {k: [] for k in ("travel", "camber", "scrub", "rc_height",
                               "damper_len", "lbj", "ubj", "cp")}
        prev_ubj = prev_rk = None
        for th in thetas:
            s = self.solve(th, prev_ubj=prev_ubj, prev_rk=prev_rk)
            prev_ubj = s["ubj"]
            cp = s["cp"]
            out["travel"].append(float(cp[1] - self.cp0[1]))
            out["scrub"].append(float(cp[0] - self.cp0[0]))
            out["camber"].append(float(s["camber"]))
            out["rc_height"].append(s["rc_height"])
            out["lbj"].append(s["lbj"]); out["ubj"].append(s["ubj"]); out["cp"].append(cp)
            if self.has_rocker:
                prev_rk = s["rk_push"]
                out["damper_len"].append(s["damper_len"])
        for k in ("travel", "camber", "scrub", "rc_height"):
            out[k] = np.asarray(out[k])
        if self.has_rocker:
            out["damper_len"] = np.asarray(out["damper_len"])
            out["motion_ratio"] = np.abs(np.gradient(out["damper_len"], out["travel"]))
        return out


# A reasonable FSAE-ish default geometry (short-long arm) with a pushrod-on-lower-
# arm + rocker + damper. Replace every point with your car's hardpoints.
DEFAULT = DoubleWishbone2D(
    lca_inboard=[0.18, 0.12],
    uca_inboard=[0.22, 0.32],
    lower_ball_joint=[0.58, 0.13],
    upper_ball_joint=[0.55, 0.34],
    contact_patch=[0.60, 0.00],
    # actuation:
    pushrod_outboard=[0.45, 0.135],   # on the lower arm
    rocker_pivot=[0.17, 0.40],        # chassis
    rocker_pushrod=[0.2787, 0.417],   # on rocker (pushrod side)
    rocker_damper=[0.1007, 0.4721],   # on rocker (damper side)
    damper_inboard=[0.03, 0.345],     # chassis
)