"""
Red Robot 2026 game supervisor (Webots).

Runs the match clock (20 s auto + 120 s teleop), tells robots the phase,
operates the pipe factories (dispense button, 3 s cooldown), watches the
end-game levers and scores everything from object positions:

  trench pipe (lying, centreline fully inside trench) ......... 10
  smokestack pipe, by stack position 0 / 1 / 2+ ............... 20 / 40 / 60
  auto: exit start zone 20, reach end of your line 20,
        first button press 10 (per alliance), +20 per pipe scored in auto
  safe-zone contact penalty ................................... -20 (3 s grace)
  RP: win 2 / tie 1, auto RP (all robots out + 1 trench pipe in auto),
      teleop RP (lever pulled in end game + 2 pipes in the smokestacks)

controllerArgs:
  --practice    no clock: robots may be driven immediately, scoring stays live
  --no-pause    keep the simulation running after the final buzzer

All geometry comes from ../../field_config.json.
"""
import json
import math
import os
import sys

from controller import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = json.load(open(os.path.join(HERE, "..", "..", "field_config.json")))

sup = Supervisor()
TS = int(sup.getBasicTimeStep())
tx = sup.getDevice("match_tx")

PRACTICE = "--practice" in sys.argv
PAUSE_AT_END = "--no-pause" not in sys.argv
TRACE = os.environ.get("RR_TRACE") is not None   # print robot + pipe positions every 0.5 s

M, PTS, AL = CFG["match"], CFG["points"], CFG["alliances"]
AUTO_S, TELE_S, ENDGAME_S = M["auto_seconds"], M["teleop_seconds"], M["endgame_seconds"]
PRE_S = 1.0
PIPE_R, PIPE_H = 0.0381, 0.0762
FP = CFG["robot_footprint"]
NAMES = ("red", "blue")
OPP = {"red": "blue", "blue": "red"}


# ------------------------------------------------------------------ geometry helpers
def in_rect(x, y, r):
    return r[0] <= x <= r[2] and r[1] <= y <= r[3]


def footprint(node, grow=0.0):
    p, o = node.getPosition(), node.getOrientation()
    pts = [(FP["xmin"] - grow, FP["ymin"] - grow), (FP["xmax"] + grow, FP["ymin"] - grow),
           (FP["xmax"] + grow, FP["ymax"] + grow), (FP["xmin"] - grow, FP["ymax"] + grow)]
    return [(p[0] + o[0] * x + o[1] * y, p[1] + o[3] * x + o[4] * y) for x, y in pts]


def _proj(pts, ax):
    v = [p[0] * ax[0] + p[1] * ax[1] for p in pts]
    return min(v), max(v)


def polys_overlap(a, b):
    """Separating-axis test for two convex quads."""
    for poly in (a, b):
        for i in range(len(poly)):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % len(poly)]
            ax = (y0 - y1, x1 - x0)
            a0, a1 = _proj(a, ax)
            b0, b1 = _proj(b, ax)
            if a1 < b0 or b1 < a0:
                return False
    return True


def rect_poly(r):
    return [(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])]


def fully_inside(poly, r):
    return all(in_rect(x, y, r) for x, y in poly)


def fully_outside(poly, r):
    return not polys_overlap(poly, rect_poly(r))


def line_end(line):
    return line["path"][-1]


# ------------------------------------------------------------------ scene objects
# every RRRobot in the world takes part; its alliance comes from customData
# ("alliance=red" / "alliance=blue"), so partner/opponent robots can simply be
# copy-pasted in the Webots scene tree.
robots = []
_kids = sup.getRoot().getField("children")
for i in range(_kids.getCount()):
    n = _kids.getMFNode(i)
    if n.getTypeName() != "RRRobot":
        continue
    cd = n.getField("customData").getSFString()
    alliance = "blue" if "alliance=blue" in cd else "red" if "alliance=red" in cd else None
    name = n.getDef() or n.getField("name").getSFString()
    if alliance is None:
        print(f"[GAME] robot {name}: customData has no alliance=red/blue - ignored for scoring")
        continue
    robots.append({"def": name, "alliance": alliance, "node": n, "exited": False, "line": False})
print("[GAME] robots: " + ", ".join(f"{r['def']} ({r['alliance']})" for r in robots), flush=True)

pipes = []
for i in range(1, 100):
    n = sup.getFromDef(f"PIPE_{i}")
    if n is None:
        break
    pipes.append(n)

buttons, levers = {}, {}
for a in NAMES:
    b = sup.getFromDef(f"{a.upper()}_BUTTON")
    buttons[a] = {"node": b, "rest_y": b.getPosition()[1] if b else 0, "down": False, "last_dispense": -99.0}
    levers[a] = sup.getFromDef(f"{a.upper()}_LEVER")
root_children = sup.getRoot().getField("children")


# ------------------------------------------------------------------ scoring
def pipe_state(n):
    p, o = n.getPosition(), n.getOrientation()
    return p, (o[2], o[5], o[8])


def evaluate():
    """Score every pipe on the field right now. Returns per-alliance dict."""
    res = {a: {"trench": 0, "stack_pts": 0, "stack_pipes": 0, "heights": []} for a in NAMES}
    upright = []
    for n in pipes:
        p, ax = pipe_state(n)
        if abs(ax[2]) < 0.3 and p[2] < 0.06:                       # lying on its side
            e1 = (p[0] + ax[0] * PIPE_H / 2, p[1] + ax[1] * PIPE_H / 2)
            e2 = (p[0] - ax[0] * PIPE_H / 2, p[1] - ax[1] * PIPE_H / 2)
            for a in NAMES:
                t = AL[a]["trench"]
                if in_rect(*e1, t) and in_rect(*e2, t):
                    res[a]["trench"] += 1
        elif abs(ax[2]) > 0.93:                                      # standing on an end
            upright.append(p)

    cy, rs = CFG["stack_center_y"], CFG["stack_radius"]
    wall_y = cy + rs                                                 # back of the recess
    for a in NAMES:
        z = AL[a]["steam_plant_zone"]
        for cx in AL[a]["stack_centers_x"]:
            col = [p for p in upright if math.hypot(p[0] - cx, p[1] - cy) < rs + PIPE_R
                   and min(AL[a]["stack_centers_x"], key=lambda c: abs(p[0] - c)) == cx]
            col.sort(key=lambda p: p[2])
            level, prev = 0, None
            for p in col:
                if prev is None:
                    ok = (abs(p[2] - PIPE_H / 2) < 0.012 and z[0] <= p[0] - PIPE_R and p[0] + PIPE_R <= z[2]
                          and z[1] <= p[1] - PIPE_R and p[1] + PIPE_R <= wall_y + 0.002)
                else:
                    ok = (abs(p[2] - (prev[2] + PIPE_H)) < 0.015 and math.hypot(p[0] - prev[0], p[1] - prev[1]) < 0.03)
                if not ok:
                    break
                res[a]["stack_pts"] += PTS["stack"][min(level, len(PTS["stack"]) - 1)]
                res[a]["stack_pipes"] += 1
                level += 1
                prev = p
            if level:
                res[a]["heights"].append(level)
    for a in NAMES:
        res[a]["pipe_pts"] = res[a]["trench"] * PTS["trench"] + res[a]["stack_pts"]
    return res


# ------------------------------------------------------------------ match state
S = {a: {"auto": 0, "auto_pipe_bonus": 0, "button_bonus": False, "penalty": 0,
         "lever": False, "auto_rp": False, "tele_rp": False, "rp": 0} for a in NAMES}
events = []
contact_timers = {}
final = None
phase = "PRACTICE" if PRACTICE else "PRE"
t_phase0 = 0.0
dispensed = 0
pending_pipes = []


def log(msg):
    t = sup.getTime()
    events.append(f"{t:6.1f}s  {msg}")
    del events[:-5]
    print(f"[GAME] {t:6.1f}s {msg}", flush=True)


def broadcast():
    tx.send(f"PHASE:{phase}".encode())


def dispense(a, now):
    global dispensed
    x = AL[a]["factory_x"]
    y = CFG["near_wall_face_y"] + 0.06
    for n in pipes:                                   # outlet blocked?
        p = n.getPosition()
        if math.hypot(p[0] - x, p[1] - y) < 0.085 and p[2] < 0.1:
            log(f"{a} factory jammed (pipe in the outlet)")
            return False
    for r in robots:
        if polys_overlap(footprint(r["node"]), rect_poly([x - 0.05, y - 0.05, x + 0.05, y + 0.05])):
            log(f"{a} factory blocked by {r['def']}")
            return False
    dispensed += 1
    v = M["dispense_speed_mps"]
    # the import is applied at the next step, so the node is picked up (and pushed
    # out of the factory) by finish_dispenses() once it exists
    root_children.importMFNodeFromString(
        -1, f'DEF DISPENSED_{dispensed} RRPipe {{ translation {x} {y} {PIPE_R + 0.002} rotation 0 1 0 1.5708 '
            f'name "dispensed pipe {dispensed}" }}')
    pending_pipes.append((f"DISPENSED_{dispensed}", v))
    log(f"{a} factory dispensed a pipe")
    return True


def finish_dispenses():
    for d, v in list(pending_pipes):
        n = sup.getFromDef(d)
        if n is None:
            continue
        n.setVelocity([0, v, 0, -v / PIPE_R, 0, 0])
        pipes.append(n)
        pending_pipes.remove((d, v))


def update_buttons(now, in_auto):
    for a in NAMES:
        b = buttons[a]
        if b["node"] is None:
            continue
        depth = b["rest_y"] - b["node"].getPosition()[1]
        if not b["down"] and depth > 0.004:
            b["down"] = True
            if in_auto and not S[a]["button_bonus"]:
                S[a]["button_bonus"] = True
                S[a]["auto"] += PTS["auto_first_button"]
                log(f"{a} auto: first button press +{PTS['auto_first_button']}")
            if now - b["last_dispense"] >= M["dispense_cooldown_s"]:
                if dispense(a, now):
                    b["last_dispense"] = now
            else:
                log(f"{a} button pressed during cooldown")
        elif b["down"] and depth < 0.002:
            b["down"] = False


def lever_pulled(a):
    n = levers[a]
    if n is None:
        return False
    return math.acos(max(-1.0, min(1.0, n.getOrientation()[8]))) > math.radians(35)


def update_auto_bonuses():
    for r in robots:
        a, node = r["alliance"], r["node"]
        fp = footprint(node)
        if not r["exited"] and fully_outside(fp, AL[a]["start_zone"]):
            r["exited"] = True
            S[a]["auto"] += PTS["auto_exit_start"]
            log(f"{r['def']} auto: left the start zone +{PTS['auto_exit_start']}")
        if not r["line"]:
            p = node.getPosition()
            for line in CFG["lines"]["list"]:
                if line["owner"] != a:
                    continue
                ex, ey = line_end(line)
                if math.hypot(p[0] - ex, p[1] - ey) < CFG["lines"]["end_radius"]:
                    r["line"] = True
                    S[a]["auto"] += PTS["auto_line_follow"]
                    log(f"{r['def']} auto: reached the end of its line +{PTS['auto_line_follow']}")
                    break


def safe_zones(a):
    return [AL[a]["safe_zone"], AL[a]["trench"], AL[a]["steam_plant_zone"]]


def robot_contacts(r):
    """Physics contact points of a robot (incl. wheels/mechanisms), floor contacts excluded."""
    return [tuple(c.point) for c in r["node"].getContactPoints(True) if c.point[2] > 0.0015]


def touching(c1, c2):
    # ODE reports the same contact point for both bodies of a contact
    return any(abs(a[0] - b[0]) < 0.004 and abs(a[1] - b[1]) < 0.004 and abs(a[2] - b[2]) < 0.004
               for a in c1 for b in c2)


def update_penalties(now):
    if len({r["alliance"] for r in robots}) < 2:
        return
    contacts = {r["def"]: robot_contacts(r) for r in robots}
    for i, r1 in enumerate(robots):
        for r2 in robots[i + 1:]:
            if r1["alliance"] == r2["alliance"]:
                continue
            if not touching(contacts[r1["def"]], contacts[r2["def"]]):
                continue
            for victim, offender in ((r1, r2), (r2, r1)):
                fp = footprint(victim["node"])
                if any(fully_inside(fp, z) for z in safe_zones(victim["alliance"])):
                    key = (offender["def"], victim["def"])
                    if now - contact_timers.get(key, -99) > 3.0:
                        S[offender["alliance"]]["penalty"] += PTS["safe_zone_contact"]
                        log(f"PENALTY {offender['def']} touched {victim['def']} in its safe zone {PTS['safe_zone_contact']}")
                    contact_timers[key] = now


def totals(ev):
    out = {}
    for a in NAMES:
        out[a] = S[a]["auto"] + S[a]["auto_pipe_bonus"] + ev[a]["pipe_pts"] + S[a]["penalty"]
    return out


def end_of_auto():
    ev = evaluate()
    for a in NAMES:
        n_scored = ev[a]["trench"] + ev[a]["stack_pipes"]
        S[a]["auto_pipe_bonus"] = n_scored * PTS["auto_pipe_bonus"]
        mine = [r for r in robots if r["alliance"] == a]
        all_out = bool(mine) and all(fully_outside(footprint(r["node"]), AL[a]["start_zone"]) for r in mine)
        S[a]["auto_rp"] = all_out and ev[a]["trench"] >= 1
        log(f"{a} end of auto: {n_scored} pipe(s) scored (+{S[a]['auto_pipe_bonus']}), auto RP "
            f"{'YES' if S[a]['auto_rp'] else 'no'}")


def finish():
    global final
    ev = evaluate()
    tot = totals(ev)
    for a in NAMES:
        S[a]["tele_rp"] = S[a]["lever"] and ev[a]["stack_pipes"] >= 2
        win = tot[a] > tot[OPP[a]]
        tie = tot[a] == tot[OPP[a]]
        S[a]["rp"] = (2 if win else 1 if tie else 0) + int(S[a]["auto_rp"]) + int(S[a]["tele_rp"])
    final = (ev, tot)
    print("\n================ FINAL ================", flush=True)
    for a in NAMES:
        e = ev[a]
        print(f"{a.upper():5s} {tot[a]:4d} pts | auto bonus {S[a]['auto']} + auto pipes {S[a]['auto_pipe_bonus']}"
              f" | trench {e['trench']}x{PTS['trench']} | stacks {e['stack_pts']} ({e['stack_pipes']} pipes, heights {e['heights']})"
              f" | penalties {S[a]['penalty']} | lever {'Y' if S[a]['lever'] else 'n'}"
              f" | RP {S[a]['rp']} (autoRP {int(S[a]['auto_rp'])}, teleRP {int(S[a]['tele_rp'])})", flush=True)
    if tot["red"] == tot["blue"]:
        print("TIE - the real game goes to sudden death (next score wins)", flush=True)
    print("=======================================\n", flush=True)


# ------------------------------------------------------------------ HUD
def hud(now, ev):
    if PRACTICE:
        clock = f"PRACTICE  {now:6.1f}s"
    elif phase == "PRE":
        clock = "GET READY"
    elif phase == "AUTO":
        clock = f"AUTONOMOUS  {max(0, AUTO_S - (now - t_phase0)):4.1f}"
    elif phase == "TELEOP":
        left = max(0, TELE_S - (now - t_phase0))
        clock = f"TELEOP  {int(left // 60)}:{int(left % 60):02d}" + ("  END GAME" if left <= ENDGAME_S else "")
    else:
        clock = "MATCH OVER"
    tot = totals(ev) if final is None else final[1]
    sup.setLabel(0, clock, 0.36, 0.005, 0.10, 0x111111, 0.0, "Arial Black")
    for i, a in enumerate(NAMES):
        e = ev[a] if final is None else final[0][a]
        txt = (f"{a.upper()}  {tot[a]}\n"
               f"auto {S[a]['auto'] + S[a]['auto_pipe_bonus']}  trench {e['trench']}\n"
               f"stacks {e['stack_pts']} {e['heights']}\n"
               f"lever {'PULLED' if S[a]['lever'] else '-'}  pen {S[a]['penalty']}"
               + (f"\n RP {S[a]['rp']}" if final else ""))
        col = 0xB3261E if a == "red" else 0x1F4FB8
        sup.setLabel(1 + i, txt, 0.005 if a == "blue" else 0.70, 0.075, 0.058, col, 0.0, "Lucida Console")
    sup.setLabel(3, "\n".join(events), 0.005, 0.86, 0.052, 0x222222, 0.0, "Lucida Console")


# ------------------------------------------------------------------ main loop
log("practice mode: no clock, drive any time" if PRACTICE else "match starting")
last_bcast = -1.0
last_score = (0, 0)
while sup.step(TS) != -1:
    now = sup.getTime()
    finish_dispenses()

    if not PRACTICE:
        el = now - t_phase0
        if phase == "PRE" and el >= PRE_S:
            phase, t_phase0 = "AUTO", now
            log("AUTONOMOUS")
        elif phase == "AUTO" and el >= AUTO_S:
            end_of_auto()
            phase, t_phase0 = "TELEOP", now
            log("TELEOP - click the 3D view, press X to take control")
        elif phase == "TELEOP" and el >= TELE_S:
            phase = "END"
            log("MATCH OVER")
            broadcast()
            finish()

    if phase == "END" or now - last_bcast >= 0.25:
        broadcast()
        last_bcast = now

    if phase in ("AUTO", "TELEOP", "PRACTICE"):
        update_buttons(now, phase == "AUTO")
        if phase == "AUTO":
            update_auto_bonuses()
        update_penalties(now)
        in_endgame = PRACTICE or (phase == "TELEOP" and now - t_phase0 >= TELE_S - ENDGAME_S)
        for a in NAMES:
            if in_endgame and not S[a]["lever"] and lever_pulled(a):
                S[a]["lever"] = True
                log(f"{a} END GAME LEVER pulled")

    if TRACE and phase in ("AUTO", "PRACTICE") and int(now / 0.5) != int((now - TS / 1000.0) / 0.5):
        for r in robots:
            p, o = r["node"].getPosition(), r["node"].getOrientation()
            print(f"[TRACE] {now:5.1f} {r['def']} x={p[0]:.3f} y={p[1]:.3f} hdg={math.degrees(math.atan2(o[3], o[0])):.1f}", flush=True)
        print("[TRACE] pipes " + " ".join(f"({n.getPosition()[0]:.2f},{n.getPosition()[1]:.2f},z{n.getPosition()[2]:.3f},up{abs(n.getOrientation()[8]):.2f})" for n in pipes[:6]), flush=True)
    ev = evaluate()
    hud(now, ev)
    if phase != "END":
        tot = totals(ev)
        snap = (tot["red"], tot["blue"])
        if snap != last_score:
            last_score = snap
            print(f"[GAME] {now:6.1f}s score  RED {tot['red']} (trench {ev['red']['trench']}, stacks {ev['red']['stack_pts']} "
                  f"{ev['red']['heights']})   BLUE {tot['blue']} (trench {ev['blue']['trench']}, stacks {ev['blue']['stack_pts']} "
                  f"{ev['blue']['heights']})", flush=True)

    if phase == "END" and PAUSE_AT_END:
        PAUSE_AT_END = False          # pause once; resuming afterwards is allowed
        sup.step(TS)
        sup.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
