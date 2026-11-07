"""
One-time CAD processing: turns the Onshape STL exports in ../cad/ into the
meshes the Webots world uses (in ../protos/meshes/) plus a JSON list of
collision boxes for the field.

Only needed again if the CAD changes.  Requires: pip install trimesh numpy
(optional: fast-simplification, to lighten the robot mesh).

Coordinate frame (same as the Onshape field assembly, metres):
  +x = toward the RED side (right when standing at the pipe factories)
  +y = toward the smokestacks
  +z = up, floor top at z = 0
"""
import json
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CAD = os.path.join(ROOT, "cad")
OUT = os.path.join(ROOT, "protos", "meshes")
os.makedirs(OUT, exist_ok=True)


def load(name):
    return trimesh.load(os.path.join(CAD, name), force="mesh")


def concat(meshes):
    meshes = [m for m in meshes if len(m.faces)]
    return trimesh.util.concatenate(meshes) if meshes else trimesh.Trimesh()


def export(mesh, name):
    mesh.export(os.path.join(OUT, name))
    print(f"  wrote {name}: {len(mesh.faces)} triangles")


def maybe_simplify(mesh, target):
    if len(mesh.faces) <= target:
        return mesh
    try:
        return mesh.simplify_quadric_decimation(face_count=target)
    except Exception as e:  # fast-simplification not installed
        print(f"  (no decimation: {e})")
        return mesh


# ---------------------------------------------------------------- field
print("Field")
field = load("Field_Assembly.stl")
bodies = field.split(only_watertight=False)

floor, chute, stacks, rest, roofs = None, None, [], [], []
for b in bodies:
    lo, hi = b.bounds
    ext = hi - lo
    if ext[0] > 2.4 and ext[1] > 2.5 and ext[2] < 0.01:
        floor = b                                   # replaced by textured floor
    elif hi[1] < -1.26 and lo[1] < -1.4:
        chute = b                                   # pipe-factory stand
    elif lo[1] > 1.17 and hi[1] < 1.225 and abs((lo[0] + hi[0]) / 2) > 0.03:
        stacks.append(b)                            # smokestack inserts
    else:
        rest.append(b)
        if ext[2] < 0.005 and lo[2] > 0.19:
            roofs.append(b)                         # 1/8" plates over the trenches

main = concat([b for b in rest if not any(b is r for r in roofs)])
export(concat(roofs), "trench_roof.stl")
export(main, "field_main.stl")
export(concat([s for s in stacks if s.bounds.mean(0)[0] < 0]), "stacks_blue.stl")
export(concat([s for s in stacks if s.bounds.mean(0)[0] > 0]), "stacks_red.stl")
chute_c = chute.copy()
chute_c.apply_translation([-chute.bounds.mean(0)[0], 0, 0])   # centre on x = 0
export(chute_c, "factory_stand.stl")

# collision boxes: bounding boxes of every real (non-degenerate) body
boxes, seen = [], set()
for b in rest:
    lo, hi = b.bounds
    ext = hi - lo
    if (ext < 0.002).any():          # stray single faces in the export
        continue
    key = tuple(np.round(np.r_[lo, hi], 3))
    if key in seen:
        continue
    seen.add(key)
    boxes.append({"center": ((lo + hi) / 2).round(5).tolist(),
                  "size": ext.round(5).tolist()})
json.dump(boxes, open(os.path.join(OUT, "field_collision_boxes.json"), "w"), indent=1)
print(f"  {len(boxes)} collision boxes")

# ---------------------------------------------------------------- pipe
print("Pipe")
pipe = load("Pipe.stl")
pipe.apply_translation(-pipe.bounds.mean(0))
# CAD axis is +y; Webots Cylinder axis is +z -> rotate +90 deg about x
pipe.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
export(pipe, "pipe.stl")

# ---------------------------------------------------------------- dispenser
print("Dispenser")
disp = load("Dispenser.stl")
# outlet points along -x in the export; turn it to +y (into the field)
disp.apply_transform(trimesh.transformations.rotation_matrix(-np.pi / 2, [0, 0, 1]))
lo, hi = disp.bounds
disp.apply_translation([-(lo[0] + hi[0]) / 2, -hi[1], 0.013 - lo[2]])
export(disp, "dispenser.stl")

# ---------------------------------------------------------------- robot
print("Robot")
robot = load("mainV2.stl")
rb = robot.split(only_watertight=False)
wheel_parts = []
for b in rb:
    lo, hi = b.bounds
    if lo[1] < -0.06 and hi[1] < -0.06 and lo[0] > 0.085 and hi[0] < 0.16:
        wheel_parts.append(b)                       # tyre + hub on -y side
body = concat([b for b in rb if not any(b is w for w in wheel_parts)])
wheel = concat(wheel_parts)
wlo, whi = wheel.bounds
wc = np.array([(wlo[0] + whi[0]) / 2, (wlo[1] + whi[1]) / 2, (wlo[2] + whi[2]) / 2])
radius = (whi[2] - wlo[2]) / 2
print(f"  wheel centre {wc.round(4)}, radius {radius:.4f}")

# The export is upside-down: flip about y (x -> -x, z -> -z) so the
# chamfered nose faces +x, then put the axle midpoint at the origin with the
# wheel centre at z = radius (robot frame: origin on the floor under the axle).
flip = trimesh.transformations.rotation_matrix(np.pi, [0, 1, 0])
for m in (body, wheel):
    m.apply_transform(flip)
wc_f = np.array([-wc[0], wc[1], -wc[2]])
shift = np.array([-wc_f[0], 0, radius - wc_f[2]])
body.apply_translation(shift)
body = maybe_simplify(body, 40000)
export(body, "robot_body.stl")
wheel.apply_translation(-np.array([-wc[0], wc[1], -wc[2]]))   # wheel centred at origin
export(wheel, "robot_wheel.stl")
lo, hi = body.bounds
info = {"wheel_radius": round(float(radius), 5),
        "wheel_y": round(float(abs(wc[1])), 5),
        "body_bounds": [lo.round(4).tolist(), hi.round(4).tolist()]}
json.dump(info, open(os.path.join(OUT, "robot_info.json"), "w"), indent=1)
print("  ", info)
