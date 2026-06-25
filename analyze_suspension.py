"""
analyze_suspension.py — sweep the wheel through its travel and measure the
kinematic outputs: camber gain, scrub (half-track change), roll center height,
and (if a rocker is defined) the motion ratio.

Run:  python analyze_suspension.py
Outputs a printed table and saves docs/suspension_kinematics.png
"""
import os
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

from kinematics import DoubleWishbone2D, DEFAULT, from_hardpoints


def load_hardpoints_csv(path):
    """Read a 'name,y,z' CSV into a {name: [y, z]} dict."""
    hp = {}
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if not row or row[0].strip().lower() in ("name", "") or row[0].startswith("#"):
                continue
            name, y, z = row[0].strip(), float(row[1]), float(row[2])
            hp[name] = [y, z]
    return hp


# Geometry: a CSV path on the command line, else the built-in DEFAULT.
#   python analyze_suspension.py mycar.csv
if len(sys.argv) > 1:
    susp = from_hardpoints(load_hardpoints_csv(sys.argv[1]))
    print(f"loaded hardpoints from {sys.argv[1]}")
else:
    susp = DEFAULT

TRAVEL = 0.030   # +/- 30 mm
N = 61

s = susp.sweep(travel=TRAVEL, n=N)
travel_mm = s["travel"] * 1000.0
scrub_mm = s["scrub"] * 1000.0
rc_mm = s["rc_height"] * 1000.0
camber = s["camber"]
has_mr = "motion_ratio" in s

i = N // 2
camber_gain = (camber[i + 1] - camber[i - 1]) / (travel_mm[i + 1] - travel_mm[i - 1])

print(f"link lengths (m): LCA={susp.lca_len:.4f}  "
      f"UCA={susp.uca_len:.4f}  upright={susp.upright_len:.4f}")
print(f"camber gain @ ride: {camber_gain:+.4f} deg/mm  ({camber_gain*25:+.3f} deg / 25 mm)")
print(f"roll center height @ ride: {rc_mm[i]:+.1f} mm")
print(f"KPI @ ride: {s['kpi'][i]:.2f} deg   "
      f"scrub radius @ ride: {s['scrub_radius'][i]*1000:+.1f} mm   "
      f"FVSA @ ride: {s['fvsa'][i]:.2f} m")
if has_mr:
    mr = s["motion_ratio"]
    print(f"motion ratio @ ride: {mr[i]:.3f} (damper/wheel)  "
          f"-> wheel/damper = {1/mr[i]:.3f},  wheel-rate factor MR^2 = {mr[i]**2:.3f}")
print()

header = " travel[mm]  camber[deg]  scrub[mm]  RC[mm]" + ("   MR(d/w)" if has_mr else "")
print(header)
for j in range(0, N, 5):
    row = f"  {travel_mm[j]:+7.1f}     {camber[j]:+7.3f}    {scrub_mm[j]:+7.2f}   {rc_mm[j]:+6.1f}"
    if has_mr:
        row += f"    {s['motion_ratio'][j]:.3f}"
    print(row)

# --- plots ----------------------------------------------------------------- #
panels = [
    (camber, "camber [deg]   (+ = top outboard)", "Camber gain", "#ff5a3c"),
    (scrub_mm, "scrub (half-track change) [mm]", "Half-track change", "#5b8def"),
    (rc_mm, "roll center height [mm]", "Roll center migration", "#3cba7a"),
]
if has_mr:
    panels.append((s["motion_ratio"], "motion ratio [damper/wheel]",
                   "Motion ratio", "#b07cff"))

cols = len(panels)
fig, axes = plt.subplots(1, cols, figsize=(5 * cols, 4.5))
for ax, (x, xlabel, title, color) in zip(np.atleast_1d(axes), panels):
    ax.plot(x, travel_mm, color=color, lw=2)
    ax.axhline(0, color="#888", lw=0.8)
    ax.axvline(0, color="#888", lw=0.8)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("wheel travel [mm]   (+ = bump)")
    ax.set_title(title)
    ax.grid(alpha=0.3)

fig.tight_layout()
os.makedirs("docs", exist_ok=True)
out = "docs/suspension_kinematics.png"
fig.savefig(out, dpi=130)
print(f"\nsaved {out}")
