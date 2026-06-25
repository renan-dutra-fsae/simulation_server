"""
scenes.py — builds `reference_engine` worlds and records their trajectories.

A "scene factory" is a function `params -> (world, manifest)`:
  * world     : a reference_engine.World, fully configured (bodies, forces, constraints)
  * manifest  : {body_name: {"radius": float, "color": str}} — viewer hints only

`record(world, manifest, ...)` is fully generic: it reads `world.bodies` and
`world.constraints`, so any scene you build with the engine serializes for free.
"""
import numpy as np

from reference_engine.world import World
from reference_engine.geometry import Point, Vector
from reference_engine.force import Gravity

INF = float("inf")


# --------------------------------------------------------------------------- #
# Generic recorder — turns a World into the JSON contract the viewer expects.
# --------------------------------------------------------------------------- #
def _is_static(body) -> bool:
    return getattr(body, "mass", 0.0) == INF


def record(world: World, manifest: dict, dt: float, n_frames: int, scene: str) -> dict:
    # Body manifest: built once from the world, enriched with viewer hints.
    bodies = []
    for b in world.bodies:
        hint = manifest.get(b.name, {})
        bodies.append({
            "id": b.name,
            "static": _is_static(b),
            "radius": float(hint.get("radius", 0.08 if not _is_static(b) else 0.05)),
            "color": hint.get("color", "#8a8f98" if _is_static(b) else "#ff5a3c"),
        })

    # Links: distance constraints become lines (rods, control arms, springs...).
    links = [
        {"from": c.p1.name, "to": c.p2.name, "kind": "constraint"}
        for c in getattr(world, "constraints", [])
    ]

    # Trajectory: step the world and snapshot every body's position.
    frames = []
    for _ in range(n_frames):
        world.step(dt)
        snapshot = {}
        for b in world.bodies:
            p = b.position  # reference_engine Point, z is "up"
            snapshot[b.name] = [float(p.x), float(p.y), float(p.z)]
        frames.append({"t": round(world.time, 6), "p": snapshot})

    return {
        "meta": {"dt": dt, "n_frames": n_frames, "up_axis": "z", "scene": scene},
        "bodies": bodies,
        "links": links,
        "frames": frames,
    }


def _set_initial_velocity(particle, velocity: Vector, dt: float):
    """The Verlet integrator infers velocity from (position - prev_position).
    Seed prev_position so the body starts with the velocity you intend."""
    v_as_point = Point(velocity.x, velocity.y, velocity.z)
    particle.prev_position = particle.position - v_as_point * dt


# --------------------------------------------------------------------------- #
# Scene: free fall (mirrors examples/01_free_fall.py)
# --------------------------------------------------------------------------- #
def build_free_fall(params: dict):
    h = float(params.get("height", 100.0))
    world = World(world_origin=np.zeros(3))
    ball = world.add_particle("ball", mass=1.0,
                              position=Point(0.0, 0.0, h),
                              velocity=Vector(0.0, 0.0, 0.0))
    g = Gravity("gravity", Vector(0.0, 0.0, -9.81))
    g.set(ball)
    world.add_force(g)
    return world, {"ball": {"radius": 2.0, "color": "#ff5a3c"}}


# --------------------------------------------------------------------------- #
# Scene: pendulum (mirrors examples/03_pendulum.py)
# --------------------------------------------------------------------------- #
def build_pendulum(params: dict):
    dt = float(params.get("dt", 0.01))
    world = World(world_origin=np.zeros(3))
    world.set_solver_iterations(int(params.get("iterations", 20)))

    anchor = world.add_particle("anchor", mass=INF,
                                position=Point(0.0, 0.0, 50.0),
                                velocity=Vector(0.0, 0.0, 0.0))
    ball = world.add_particle("ball", mass=10.0,
                              position=Point(10.0, 0.0, 40.0),
                              velocity=Vector(0.0, 0.0, 0.0))
    g = Gravity("gravity", Vector(0.0, 0.0, -9.81))
    g.set(ball)
    world.add_force(g)

    rod_length = anchor.position.distance_to(ball.position)
    world.add_position_constraint("rod", anchor, ball, rod_length)

    _set_initial_velocity(ball, Vector(-1.0, 0.0, 0.0), dt)
    return world, {
        "anchor": {"radius": 1.5, "color": "#8a8f98"},
        "ball": {"radius": 3.0, "color": "#ff5a3c"},
    }


# --------------------------------------------------------------------------- #
# Scene: double-wishbone (planar template — replace geometry with your data)
# --------------------------------------------------------------------------- #
def build_suspension(params: dict):
    """A quarter-car-ish linkage in the y-z plane (front view).

    Chassis pickups are fixed (mass=inf); the upright's ball joints are
    particles held at arm length by distance constraints. This is a *scaffold*:
    swap in your real hardpoint coordinates and arm lengths from the FSAE car.
    """
    world = World(world_origin=np.zeros(3))
    world.set_solver_iterations(int(params.get("iterations", 30)))

    # Chassis pickup points (fixed). Coordinates in [x, y, z], z up.
    lca_chassis = world.add_particle("lca_chassis", mass=INF,
                                     position=Point(0.0, 0.20, 0.15),
                                     velocity=Vector(0, 0, 0))
    uca_chassis = world.add_particle("uca_chassis", mass=INF,
                                     position=Point(0.0, 0.25, 0.35),
                                     velocity=Vector(0, 0, 0))

    # Upright ball joints (free). Start roughly outboard of the pickups.
    lower_bj = world.add_particle("lower_bj", mass=2.0,
                                  position=Point(0.0, 0.62, 0.18),
                                  velocity=Vector(0, 0, 0))
    upper_bj = world.add_particle("upper_bj", mass=2.0,
                                  position=Point(0.0, 0.58, 0.33),
                                  velocity=Vector(0, 0, 0))

    g = Gravity("gravity", Vector(0.0, 0.0, -9.81))
    g.set(lower_bj)
    g.set(upper_bj)
    world.add_force(g)

    lca_len = lca_chassis.position.distance_to(lower_bj.position)  # lower arm
    uca_len = uca_chassis.position.distance_to(upper_bj.position)  # upper arm
    upright = lower_bj.position.distance_to(upper_bj.position)     # upright body

    world.add_position_constraint("lca", lca_chassis, lower_bj, lca_len)
    world.add_position_constraint("uca", uca_chassis, upper_bj, uca_len)
    world.add_position_constraint("upright", lower_bj, upper_bj, upright)

    return world, {
        "lca_chassis": {"radius": 0.012, "color": "#5b8def"},
        "uca_chassis": {"radius": 0.012, "color": "#5b8def"},
        "lower_bj": {"radius": 0.015, "color": "#ff5a3c"},
        "upper_bj": {"radius": 0.015, "color": "#ff5a3c"},
    }


SCENES = {
    "free_fall": build_free_fall,
    "pendulum": build_pendulum,
    "suspension": build_suspension,
}
