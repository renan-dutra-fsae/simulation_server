"""
simulation_server — a thin HTTP layer over reference_engine.

It builds a `World` from a named scene, steps it `n_frames` times, and returns
the whole trajectory as JSON. The browser (physics_viewer) plays it back.

The contract is intentionally small (see scenes.record):

    {
      "meta":   {"dt": 0.01, "n_frames": 500, "up_axis": "z", "scene": "pendulum"},
      "bodies": [{"id": "ball", "static": false, "radius": 0.15, "color": "#ff5a3c"}, ...],
      "links":  [{"from": "anchor", "to": "ball", "kind": "constraint"}, ...],
      "frames": [{"t": 0.0, "p": {"ball": [10, 0, 40], "anchor": [0, 0, 50]}}, ...]
    }

`bodies` and `links` are the *manifest*: the viewer builds meshes from it once.
`frames` is just positions per body per timestep — cheap to send and to render.
"""
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from scenes import SCENES, TRAJECTORY_SCENES, record

app = FastAPI(title="simulation_server", version="0.1.0")

# Dev-friendly CORS so physics_viewer can run from any local origin
# (vite, `python -m http.server`, file://, etc.) and still hit this API.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class SimRequest(BaseModel):
    scene: str = Field("pendulum", description="Scene name from /api/scenes")
    dt: float = Field(0.01, gt=0, le=1.0)
    n_frames: int = Field(600, gt=0, le=20000)
    params: dict = Field(default_factory=dict, description="Per-scene knobs")


def _all_scenes():
    return sorted(set(SCENES) | set(TRAJECTORY_SCENES))


@app.get("/api/scenes")
def list_scenes():
    """Names the viewer can ask for."""
    return {"scenes": _all_scenes()}


@app.post("/api/simulate")
def simulate(req: SimRequest):
    # Trajectory (prescribed-motion) scenes return a finished payload directly.
    if req.scene in TRAJECTORY_SCENES:
        params = {**req.params, "n_frames": req.n_frames}
        return TRAJECTORY_SCENES[req.scene](params)
    # Dynamic scenes are time-stepped reference_engine worlds.
    if req.scene in SCENES:
        world, manifest = SCENES[req.scene](req.params)
        return record(world, manifest, dt=req.dt, n_frames=req.n_frames, scene=req.scene)
    raise HTTPException(404, f"unknown scene '{req.scene}'")


# Convenience GET so you can hit it straight from the address bar while testing:
#   http://localhost:8000/api/simulate?scene=pendulum
@app.get("/api/simulate")
def simulate_get(scene: str = "pendulum", dt: float = 0.01, n_frames: int = 600):
    return simulate(SimRequest(scene=scene, dt=dt, n_frames=n_frames))


@app.get("/")
def health():
    return {"ok": True, "scenes": _all_scenes()}