"""
Generates worlds/red_robot_2026.wbt and worlds/textures/field_floor.png from
field_config.json + the processed meshes.  Run it after editing the config:

    python3 tools/build_world.py

Needs only Python 3 + Pillow (pip install pillow).
"""
import json
import math
import os

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CFG = json.load(open(os.path.join(ROOT, "field_config.json")))
BOXES = json.load(open(os.path.join(ROOT, "protos", "meshes", "field_collision_boxes.json")))

FLOOR_X = (-1.2192, 1.2192)
FLOOR_Y = (-1.27, 1.27)
TEX_SIZE = 2048   # power of two so Webots does not rescale it (stretched onto the 2.44 x 2.54 m floor)


def bezier(p, n=60):
    (x0, y0), (x1, y1), (x2, y2), (x3, y3) = p
    out = []
    for i in range(n + 1):
        t = i / n
        a, b, c, d = (1 - t) ** 3, 3 * t * (1 - t) ** 2, 3 * t * t * (1 - t), t ** 3
        out.append((a * x0 + b * x1 + c * x2 + d * x3, a * y0 + b * y1 + c * y2 + d * y3))
    return out


def line_points(line):
    return bezier(line["path"]) if line.get("bezier") else [tuple(p) for p in line["path"]]


# ------------------------------------------------------------------ floor texture
def make_floor_texture(path):
    w = h = TEX_SIZE
    sx = TEX_SIZE / (FLOOR_X[1] - FLOOR_X[0])   # px per metre along x
    sy = TEX_SIZE / (FLOOR_Y[1] - FLOOR_Y[0])   # px per metre along y
    img = Image.new("RGB", (w, h), (214, 214, 210))
    d = ImageDraw.Draw(img, "RGBA")

    def px(x, y):  # image row 0 = +y edge (smokestacks), so the picture reads like the manual
        return ((x - FLOOR_X[0]) * sx, (FLOOR_Y[1] - y) * sy)

    tape = CFG["tape_width"] * (sx + sy) / 2

    def rect(r, rgb, fill_alpha):
        x0, y0 = px(r[0], r[3])
        x1, y1 = px(r[2], r[1])
        if fill_alpha:
            d.rectangle([x0, y0, x1, y1], fill=(*rgb, fill_alpha))
        d.rectangle([x0, y0, x1, y1], outline=(*rgb, 255), width=int(tape))

    for name, a in CFG["alliances"].items():
        rgb = tuple(int(c * 255) for c in a["color"])
        rect(a["safe_zone"], rgb, 70)
        rect(a["start_zone"], rgb, 90)
        rect(a["trench"], rgb, 90)
        rect(a["steam_plant_zone"], rgb, 90)

    for line in CFG["lines"]["list"]:
        pts = [px(*p) for p in line_points(line)]
        d.line(pts, fill=(25, 25, 25, 255), width=int(tape), joint="curve")
        r = tape / 2
        for p in (pts[0], pts[-1]):
            d.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill=(25, 25, 25, 255))

    os.makedirs(os.path.dirname(path), exist_ok=True)
    img.save(path)
    print("wrote", path, img.size)


# ------------------------------------------------------------------ tape (geometry)
# Webots IR sensors read the *material colour* of what the ray hits, not the
# floor texture, so every piece of tape is also a 0.3 mm thick coloured strip.
TAPE_T = 0.0003


def stack_wall_boxes():
    """Box-only collision for the smokestack walls (replaces the ~19k-triangle mesh,
    which makes ODE unstable against the cylinder pipes). Per alliance: a slab
    y = face..back with a semicircular recess (6 flat segments) at every stack."""
    face, back = CFG["back_wall_face_y"], 1.2192
    cy, r = CFG["stack_center_y"], CFG["stack_radius"]
    H, zc = 0.254, 0.127
    half_open = math.sqrt(r * r - (face - cy) ** 2)          # recess half-width at the face
    a0 = math.atan2(face - cy, half_open)                     # arc start angle
    out = []
    for a in CFG["alliances"].values():
        xs = sorted(a["stack_centers_x"])
        lo = a["steam_plant_zone"][0]
        hi = a["steam_plant_zone"][2]
        edges = [lo]
        for cx in xs:
            edges += [cx - half_open, cx + half_open]
        edges.append(hi)
        for x0, x1 in zip(edges[0::2], edges[1::2]):          # solid slab between recesses
            if x1 - x0 > 1e-4:
                out.append(((x0 + x1) / 2, (face + back) / 2, zc, x1 - x0, back - face, H, 0.0))
        for cx in xs:
            out.append((cx, (cy + r + back) / 2, zc, 2 * half_open, back - (cy + r), H, 0.0))   # behind recess
            n, t = 6, 0.006
            for k in range(n):
                th = a0 + (math.pi - 2 * a0) * (k + 0.5) / n
                seg = 2 * (r + t / 2) * math.tan((math.pi - 2 * a0) / n / 2) + 0.002
                px, py = cx + (r + t / 2) * math.cos(th), cy + (r + t / 2) * math.sin(th)
                out.append((px, py, zc, seg, t, H, th - math.pi / 2))
    return out


def tape_segments():
    """Yield (x, y, length, angle_rad, rgb) strips for all lines and zone borders."""
    w = CFG["tape_width"]
    for line in CFG["lines"]["list"]:
        pts = line_points(line)
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            L = math.hypot(x1 - x0, y1 - y0)
            yield ((x0 + x1) / 2, (y0 + y1) / 2, L + w * 0.6, math.atan2(y1 - y0, x1 - x0), (0.06, 0.06, 0.06))
    for a in CFG["alliances"].values():
        rgb = tuple(a["color"])
        for key in ("start_zone", "trench", "steam_plant_zone", "safe_zone"):
            x0, y0, x1, y1 = a[key]
            h = w / 2   # tape lies inside the rectangle edge
            yield ((x0 + x1) / 2, y0 + h, x1 - x0, 0.0, rgb)
            yield ((x0 + x1) / 2, y1 - h, x1 - x0, 0.0, rgb)
            yield (x0 + h, (y0 + y1) / 2, y1 - y0 - 2 * w, math.pi / 2, rgb)
            yield (x1 - h, (y0 + y1) / 2, y1 - y0 - 2 * w, math.pi / 2, rgb)


def tape_solid():
    # one static Solid per strip: the IR colour is taken from the Solid's own Shape
    w = CFG["tape_width"]
    out = ["DEF TAPE Pose {", "  children ["]
    for i, (x, y, L, ang, rgb) in enumerate(tape_segments()):
        out.append(f"    Solid {{ translation {x:.5f} {y:.5f} {TAPE_T / 2:.5f} rotation 0 0 1 {ang:.5f} name \"tape {i}\" locked TRUE "
                   f"children [ DEF TS{i} Shape {{ castShadows FALSE appearance PBRAppearance {{ baseColor {rgb[0]} {rgb[1]} {rgb[2]} roughness 0.5 metalness 0 }} "
                   f"geometry Box {{ size {L:.5f} {w} {TAPE_T} }} }} ] boundingObject USE TS{i} }}")
    out += ["  ]", "}"]
    return "\n".join(out)


# ------------------------------------------------------------------ world file
def rot_z(deg):
    return f"0 0 1 {math.radians(deg):.5f}"


def world():
    o = []
    A = o.append
    A("#VRML_SIM R2025a utf8")
    A("# GENERATED by tools/build_world.py from field_config.json - edit the config, not this file.")
    A('IMPORTABLE EXTERNPROTO "../protos/RRPipe.proto"')   # importable: the factories spawn pipes
    A('EXTERNPROTO "../protos/RRRobot.proto"')
    A("""
WorldInfo {
  info [ "Red Robot 2026 - CMU Roboclub hackathon game simulation" ]
  title "Red Robot 2026"
  basicTimeStep 8
  FPS 30
  contactProperties [
    ContactProperties { material1 "wheel"  coulombFriction [ 1.1 ] softCFM 0.0001 }
    ContactProperties { material1 "caster" coulombFriction [ 0.05 ] }
    ContactProperties { material1 "pipe"   coulombFriction [ 0.5 ] bounce 0.05 }
    ContactProperties { material1 "pipe" material2 "pipe"  coulombFriction [ 0.6 ] bounce 0.02 }
    ContactProperties { material1 "pipe" material2 "wheel" coulombFriction [ 0.8 ] }
    ContactProperties { material1 "pipe" material2 "caster" coulombFriction [ 0.1 ] }
  ]
}
Viewpoint {
  orientation -0.33276 0.33276 0.88235 1.69564
  position 0.0 -3.05 2.55
  near 0.05
  follow "RED1"
  followType "None"
}
Background { skyColor [ 0.78 0.82 0.88 ] }
DirectionalLight { direction 0.35 0.55 -1 intensity 1.6 castShadows TRUE }
DirectionalLight { direction -0.5 -0.3 -0.7 intensity 0.6 }
""")
    # floor
    fx = FLOOR_X[1] - FLOOR_X[0]
    fy = FLOOR_Y[1] - FLOOR_Y[0]
    A(f"""DEF FLOOR Solid {{
  name "field floor"
  children [
    DEF FLOOR_SHAPE Shape {{
      appearance PBRAppearance {{
        baseColorMap ImageTexture {{ url [ "textures/field_floor.png" ] filtering 4 }}
        roughness 0.9 metalness 0
      }}
      geometry Plane {{ size {fx:.4f} {fy:.4f} }}
    }}
  ]
  boundingObject USE FLOOR_SHAPE
  locked TRUE
}}""")
    A(tape_solid())
    # walls / structure
    box_lines = "\n".join(
        f"      Pose {{ translation {b['center'][0]} {b['center'][1]} {b['center'][2]} children [ Box {{ size {b['size'][0]} {b['size'][1]} {b['size'][2]} }} ] }}"
        for b in BOXES)
    stack_boxes = "\n".join(
        f"      Pose {{ translation {x:.5f} {y:.5f} {z:.4f} rotation 0 0 1 {ang:.5f} children [ Box {{ size {sx:.5f} {sy:.5f} {sz:.4f} }} ] }}"
        for x, y, z, sx, sy, sz, ang in stack_wall_boxes())
    A(f"""DEF FIELD Solid {{
  name "field structure"
  children [
    Shape {{
      castShadows FALSE
      appearance PBRAppearance {{ baseColor 0.80 0.80 0.80 roughness 0.7 metalness 0 }}
      geometry Mesh {{ url "../protos/meshes/field_main.stl" }}
    }}
    Shape {{
      castShadows FALSE
      appearance PBRAppearance {{ baseColor 0.85 0.9 0.95 roughness 0.1 metalness 0 transparency 0.75 }}
      geometry Mesh {{ url "../protos/meshes/trench_roof.stl" }}
    }}
    Shape {{
      castShadows FALSE
      appearance PBRAppearance {{ baseColor {' '.join(map(str, CFG['alliances']['blue']['color']))} roughness 0.6 metalness 0 }}
      geometry DEF STACKS_BLUE Mesh {{ url "../protos/meshes/stacks_blue.stl" }}
    }}
    Shape {{
      castShadows FALSE
      appearance PBRAppearance {{ baseColor {' '.join(map(str, CFG['alliances']['red']['color']))} roughness 0.6 metalness 0 }}
      geometry DEF STACKS_RED Mesh {{ url "../protos/meshes/stacks_red.stl" }}
    }}
  ]
  boundingObject Group {{
    children [
{box_lines}
{stack_boxes}
    ]
  }}
  locked TRUE
}}""")

    # pipe factories (visual), dispense buttons, levers
    yw = CFG["near_wall_face_y"]
    for name, a in CFG["alliances"].items():
        col = " ".join(map(str, a["color"]))
        U = name.upper()
        fxp = a["factory_x"]
        A(f"""Pose {{
  translation {fxp} 0 0
  children [
    Shape {{ castShadows FALSE appearance PBRAppearance {{ baseColor 0.55 0.55 0.58 roughness 0.6 metalness 0 }} geometry Mesh {{ url "../protos/meshes/factory_stand.stl" }} }}
    Pose {{ translation 0 -1.27 0 children [
      Shape {{ castShadows FALSE appearance PBRAppearance {{ baseColor {col} roughness 0.5 metalness 0 }} geometry Mesh {{ url "../protos/meshes/dispenser.stl" }} }}
    ] }}
  ]
}}""")
        bx = a["button_x"]
        A(f"""Solid {{
  translation {bx} {yw} 0
  name "{name} dispense button"
  children [
    Pose {{ translation 0 0.008 0.035 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.15 0.15 0.15 roughness 0.6 metalness 0 }} geometry Box {{ size 0.07 0.016 0.05 }} }} ] }}
    SliderJoint {{
      jointParameters JointParameters {{ axis 0 -1 0 springConstant 60 dampingConstant 3 minStop 0 maxStop 0.012 }}
      endPoint DEF {U}_BUTTON Solid {{
        translation 0 0.027 0.035
        name "{name} button plunger"
        children [ Shape {{ appearance PBRAppearance {{ baseColor {col} roughness 0.4 metalness 0 emissiveColor {' '.join(str(c * 0.25) for c in a['color'])} }} geometry Box {{ size 0.05 0.02 0.035 }} }} ]
        boundingObject Box {{ size 0.05 0.02 0.035 }}
        physics Physics {{ density -1 mass 0.02 }}
      }}
    }}
  ]
  boundingObject Pose {{ translation 0 0.008 0.035 children [ Box {{ size 0.07 0.016 0.05 }} ] }}
}}""")
        lx, ly = a["lever_xy"]
        A(f"""Solid {{
  translation {lx} {ly} 0
  name "{name} end game lever"
  children [
    Pose {{ translation 0 0 0.015 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.2 0.2 0.2 roughness 0.6 metalness 0 }} geometry Box {{ size 0.05 0.05 0.03 }} }} ] }}
    HingeJoint {{
      jointParameters HingeJointParameters {{ axis 1 0 0 anchor 0 0 0.03 springConstant 0.15 dampingConstant 0.02 minStop -1.3 maxStop 1.3 }}
      endPoint DEF {U}_LEVER Solid {{
        translation 0 0 0.03
        name "{name} lever arm"
        children [
          Pose {{ translation 0 0 0.055 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.75 0.75 0.75 roughness 0.3 metalness 0.8 }} geometry Box {{ size 0.025 0.025 0.11 }} }} ] }}
          Pose {{ translation 0 0 0.115 children [ Shape {{ appearance PBRAppearance {{ baseColor {col} roughness 0.4 metalness 0 }} geometry Sphere {{ radius 0.015 }} }} ] }}
        ]
        boundingObject Pose {{ translation 0 0 0.062 children [ Box {{ size 0.03 0.03 0.12 }} ] }}
        physics Physics {{ density -1 mass 0.08 }}
      }}
    }}
  ]
  boundingObject Pose {{ translation 0 0 0.015 children [ Box {{ size 0.05 0.05 0.03 }} ] }}
}}""")

    # starting pipes
    for i, (x, y) in enumerate(CFG["start_pipes"]["list"], 1):
        A(f'DEF PIPE_{i} RRPipe {{ translation {x} {y} 0.0386 name "pipe {i}" }}')   # just above the tape

    # robots
    for r in CFG["robots"]:
        col = " ".join(map(str, CFG["alliances"][r["alliance"]]["color"]))
        custom = f"alliance={r['alliance']}" + (";keyboard" if r.get("keyboard") else "")
        A(f"""DEF {r['def']} RRRobot {{
  translation {r['x']} {r['y']} 0.0005
  rotation {rot_z(r['heading_deg'])}
  name "{r['def']}"
  controller "{r.get('controller', 'red_robot')}"
  customData "{custom}"
  bodyColor {col}
}}""")

    # game supervisor
    A("""DEF GAME Robot {
  name "game supervisor"
  controller "game_supervisor"
  supervisor TRUE
  children [ Emitter { name "match_tx" channel 7 range -1 } ]
}""")
    return "\n".join(o) + "\n"


if __name__ == "__main__":
    make_floor_texture(os.path.join(ROOT, "worlds", "textures", "field_floor.png"))
    out = os.path.join(ROOT, "worlds", "red_robot_2026.wbt")
    open(out, "w").write(world())
    print("wrote", out)
