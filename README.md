# simulation_server

A thin HTTP layer over [`reference-engine`](https://github.com/renan-dutra-fsae/reference-engine).
It builds a `World` from a named scene, steps it, and returns the full trajectory
as JSON for [`physics_viewer`](https://github.com/renan-dutra-fsae/physics_viewer)
to play back in the browser.

## Run

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
pip install -e ../reference-engine        # the simulation core
uvicorn app:app --reload --port 8000
```

Then open `http://localhost:8000/api/simulate?scene=pendulum` to see raw JSON,
or point the viewer at `http://localhost:8000`.

## API

| Method | Path             | Body / query                          | Returns                |
|--------|------------------|---------------------------------------|------------------------|
| GET    | `/api/scenes`    | —                                     | `{"scenes": [...]}`    |
| POST   | `/api/simulate`  | `{scene, dt, n_frames, params}`       | trajectory (see below) |
| GET    | `/api/simulate`  | `?scene=&dt=&n_frames=`               | trajectory             |

### Trajectory contract

```json
{
  "meta":   {"dt": 0.01, "n_frames": 600, "up_axis": "z", "scene": "pendulum"},
  "bodies": [{"id": "ball", "static": false, "radius": 3.0, "color": "#ff5a3c"}],
  "links":  [{"from": "anchor", "to": "ball", "kind": "constraint"}],
  "frames": [{"t": 0.0, "p": {"ball": [10, 0, 40], "anchor": [0, 0, 50]}}]
}
```

`bodies` + `links` are the manifest (built once → meshes/lines). `frames` is just
positions per body per timestep. **z is up** — the viewer is configured to match.

## Adding a scene

Write a factory `params -> (world, manifest)` in `scenes.py` using the engine,
register it in the `SCENES` dict, and it serializes automatically. See
`build_suspension` for a double-wishbone starting point.