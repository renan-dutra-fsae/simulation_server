"""
scenes.py — builds trajectories for the viewer.

Two kinds of scene:
  * DYNAMIC scenes (free_fall, pendulum): a reference_engine.World stepped in time.
    Factory signature: params -> (world, manifest); recorded by `record()`.
  * TRAJECTORY scenes (suspension): a prescribed-motion kinematic sweep that returns
    a finished payload directly. Factory signature: params -> payload.

Both emit the same JSON contract the viewer consumes:
  {meta, bodies:[{id,static,radius,color}], links:[{from,to,kind}], frames:[{t,p:{id:[x,y,z]}}]}
"""
import numpy as np

from reference_engine.world import World
from reference_engine.geometry import Point, Vector
from reference_engine.force import Gravity

from kinematics import DoubleWishbone2D, DEFAULT as SUSP_DEFAULT

INF = float("inf")


# =========================================================================== #
# Dynamic scenes (time-stepped reference_engine worlds)
# =========================================================================== #
def _is_static(body):
    return getattr(body, "mass", 0.0) == INF


def record(world: World, manifest: dict, dt: float, n_frames: int, scene: str) -> dict:
    bodies = []
    for b in world.bodies:
        hint = manifest.get(b.name, {})
        bodies.append({
            "id": b.name,
            "static": _is_static(b),
            "radius": float(hint.get("radius", 0.05 if _is_static(b) else 0.08)),
            "color": hint.get("color", "#8a8f98" if _is_static(b) else "#ff5a3c"),
        })
    links = [
        {"from": c.p1.name, "to": c.p2.name, "kind": "constraint"}
        for c in getattr(world, "constraints", [])
    ]
    frames = []
    for _ in range(n_frames):
        world.step(dt)
        snap = {b.name: [float(b.position.x), float(b.position.y), float(b.position.z)]
                for b in world.bodies}
        frames.append({"t": round(world.time, 6), "p": snap})
    return {
        "meta": {"dt": dt, "n_frames": n_frames, "up_axis": "z", "scene": scene},
        "bodies": bodies, "links": links, "frames": frames,
    }


def _set_initial_velocity(particle, velocity: Vector, dt: float):
    v = Point(velocity.x, velocity.y, velocity.z)
    particle.prev_position = particle.position - v * dt


def build_free_fall(params):
    h = float(params.get("height", 100.0))
    world = World(world_origin=np.zeros(3))
    ball = world.add_particle("ball", 1.0, Point(0.0, 0.0, h), Vector(0.0, 0.0, 0.0))
    g = Gravity("gravity", Vector(0.0, 0.0, -9.81)); g.set(ball); world.add_force(g)
    return world, {"ball": {"radius": 2.0, "color": "#ff5a3c"}}


def build_pendulum(params):
    dt = float(params.get("dt", 0.01))
    world = World(world_origin=np.zeros(3))
    world.set_solver_iterations(int(params.get("iterations", 20)))
    anchor = world.add_particle("anchor", INF, Point(0.0, 0.0, 50.0), Vector(0, 0, 0))
    ball = world.add_particle("ball", 10.0, Point(10.0, 0.0, 40.0), Vector(0, 0, 0))
    g = Gravity("gravity", Vector(0.0, 0.0, -9.81)); g.set(ball); world.add_force(g)
    world.add_position_constraint("rod", anchor, ball,
                                  anchor.position.distance_to(ball.position))
    _set_initial_velocity(ball, Vector(-1.0, 0.0, 0.0), dt)
    return world, {"anchor": {"radius": 1.5, "color": "#8a8f98"},
                   "ball": {"radius": 3.0, "color": "#ff5a3c"}}


# =========================================================================== #
# Trajectory scene: double-wishbone KINEMATIC sweep (prescribed motion)
# =========================================================================== #
def _p3(p2):
    """[y, z] (front view) -> [x=0, y, z] for the z-up viewer."""
    return [0.0, float(p2[0]), float(p2[1])]


def build_suspension_kinematics(params) -> dict:
    """Sweep the wheel through +/- travel and animate the linkage. No forces:
    this is a geometric mechanism solve, the correct way to read kinematics."""
    susp = SUSP_DEFAULT
    travel = float(params.get("travel", 0.030))      # +/- metres
    n = int(params.get("n_frames", 90))

    # Static sweep (rebound -> bump) for the reported curves
    sweep = susp.sweep(travel=travel, n=max(21, n // 2))

    # Animation: ping-pong bump<->rebound so it loops smoothly in the viewer
    th_lo = susp._theta_for_travel(-travel)
    th_hi = susp._theta_for_travel(+travel)
    half = np.linspace(th_lo, th_hi, n // 2)
    thetas = np.concatenate([half, half[::-1]])

    statics = {"lca_in": susp.lca_in, "uca_in": susp.uca_in}
    frames = []
    prev = prev_rk = None
    for k, th in enumerate(thetas):
        s = susp.solve(th, prev_ubj=prev, prev_rk=prev_rk)
        prev = s["ubj"]
        p = {
            "lca_in": _p3(susp.lca_in), "uca_in": _p3(susp.uca_in),
            "lbj": _p3(s["lbj"]), "ubj": _p3(s["ubj"]), "contact": _p3(s["cp"]),
        }
        if susp.has_rocker:
            prev_rk = s["rk_push"]
            p.update({
                "rk_piv": _p3(susp.rk_piv), "dmp_in": _p3(susp.dmp_in),
                "pr_out": _p3(s["pr_out"]), "rk_push": _p3(s["rk_push"]),
                "rk_damp": _p3(s["rk_damp"]),
            })
        frames.append({"t": round(k / 60.0, 4), "p": p})

    bodies = [
        {"id": "lca_in", "static": True, "radius": 0.012, "color": "#5b8def"},
        {"id": "uca_in", "static": True, "radius": 0.012, "color": "#5b8def"},
        {"id": "lbj", "static": False, "radius": 0.015, "color": "#ff5a3c"},
        {"id": "ubj", "static": False, "radius": 0.015, "color": "#ff5a3c"},
        {"id": "contact", "static": False, "radius": 0.010, "color": "#e6e8eb"},
    ]
    links = [
        {"from": "lca_in", "to": "lbj", "kind": "lower_arm"},
        {"from": "uca_in", "to": "ubj", "kind": "upper_arm"},
        {"from": "lbj", "to": "ubj", "kind": "upright"},
        {"from": "lbj", "to": "contact", "kind": "spindle"},
        {"from": "ubj", "to": "contact", "kind": "spindle"},
    ]
    if susp.has_rocker:
        bodies += [
            {"id": "rk_piv", "static": True, "radius": 0.012, "color": "#5b8def"},
            {"id": "dmp_in", "static": True, "radius": 0.012, "color": "#5b8def"},
            {"id": "pr_out", "static": False, "radius": 0.010, "color": "#b07cff"},
            {"id": "rk_push", "static": False, "radius": 0.010, "color": "#b07cff"},
            {"id": "rk_damp", "static": False, "radius": 0.010, "color": "#b07cff"},
        ]
        links += [
            {"from": "pr_out", "to": "rk_push", "kind": "pushrod"},
            {"from": "rk_piv", "to": "rk_push", "kind": "rocker"},
            {"from": "rk_piv", "to": "rk_damp", "kind": "rocker"},
            {"from": "rk_damp", "to": "dmp_in", "kind": "damper"},
        ]

    kin = {
        "travel_mm": (sweep["travel"] * 1000).round(3).tolist(),
        "camber_deg": sweep["camber"].round(4).tolist(),
        "scrub_mm": (sweep["scrub"] * 1000).round(3).tolist(),
        "rc_height_mm": (sweep["rc_height"] * 1000).round(3).tolist(),
    }
    if "motion_ratio" in sweep:
        kin["motion_ratio"] = sweep["motion_ratio"].round(4).tolist()

    return {
        "meta": {
            "dt": 1 / 60.0, "n_frames": len(frames), "up_axis": "z", "scene": "suspension",
            "kinematics": kin,
        },
        "bodies": bodies, "links": links, "frames": frames,
    }


# Registries -------------------------------------------------------------- #
SCENES = {            # dynamic: (world, manifest) -> record()
    "free_fall": build_free_fall,
    "pendulum": build_pendulum,
}
TRAJECTORY_SCENES = {  # prescribed: params -> finished payload
    "suspension": build_suspension_kinematics,
}