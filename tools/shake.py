#!/usr/bin/env python3
"""Independent physical check of a packing result: a shake test in a physics engine.

Unlike the validator, this does not use the packing engine's stability formulas. The load is
rebuilt as rigid boxes in PyBullet, then pushed with a horizontal acceleration in each of the
four directions, as on the EUMOS 40509 test bench (0.5 g; 0.8 g forwards in vehicles, per
EN 12195-1). A box that turns by more than MAX_TILT_DEG or moves by more than MAX_SHIFT_MM
counts as fallen (sliding a little inside the wrapped unit is tolerated).

Stretch wrap on a pallet is modelled as walls around the perimeter: it keeps boxes from
falling out, never from tipping inwards. Vehicles have walls; the doors (rear face) count as
secured by a strap or bar only if --rear-bar is given.

usage: shake.py request.json result.json [--bin N] [--rear-bar] [--json out.json]
needs: pip install pybullet numpy
"""
import argparse
import json
import math
import sys

import pybullet as pb

G = 9.81
FRICTION = 0.6  # cardboard on cardboard / wood, static (EN 12195-1 Annex B: 0.5–0.6)
BODY_FRICTION = FRICTION ** 0.5  # PyBullet multiplies the two bodies' coefficients
MAX_TILT_DEG = 10.0
MAX_SHIFT_MM = 100.0
RAMP_S = 0.4
HOLD_S = 1.2
DT = 1.0 / 480.0


def build(place, bin_, rear_bar):
    pb.resetSimulation()
    pb.setPhysicsEngineParameter(fixedTimeStep=DT, numSolverIterations=150, numSubSteps=1)
    mm = 0.001
    base = bin_["base_z"] * mm
    w, d = place["width"] * mm, place["depth"] * mm
    top = max((p["z"] + p["height"] for p in bin_["placed_items"]), default=0) * mm + 0.05

    def static_box(cx, cy, cz, hx, hy, hz):
        col = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[hx, hy, hz])
        b = pb.createMultiBody(0, col, basePosition=[cx, cy, cz])
        pb.changeDynamics(b, -1, lateralFriction=BODY_FRICTION, collisionMargin=0.0005)
        return b

    # Deck (pallet top or floor), much larger than the load.
    static_box(w / 2, d / 2, base - 0.05, w, d, 0.05)
    t = 0.05
    hz = (top - base) / 2
    zc = base + hz
    is_pallet = place["type"] == "PALLET"
    wrapped = place.get("stretch_wrapped")
    wrapped = True if wrapped is None and is_pallet else bool(wrapped)
    walls = []
    if not is_pallet or wrapped:
        walls += [(-t, d / 2, t, d), (w + t, d / 2, t, d), (w / 2, -t, w, t)]  # -x, +x, -y (front)
        if is_pallet or rear_bar:
            walls.append((w / 2, d + t, w, t))  # +y (film / doors with strap or bar)
    for cx, cy, hx, hy in walls:
        static_box(cx, cy, zc, hx, hy, hz)

    bodies = []
    for p in bin_["placed_items"]:
        hx, hy, hh = p["width"] * mm / 2, p["depth"] * mm / 2, p["height"] * mm / 2
        col = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[hx, hy, hh])
        pos = [p["x"] * mm + hx, p["y"] * mm + hy, p["z"] * mm + hh]
        b = pb.createMultiBody(max(p["weight"], 0.05), col, basePosition=pos)
        # Bullet's default 40 mm collision margin rounds small boxes into pebbles.
        pb.changeDynamics(b, -1, lateralFriction=BODY_FRICTION, restitution=0.0, linearDamping=0.04,
                          angularDamping=0.04, collisionMargin=0.0005)
        bodies.append((p, b, pos))
    return bodies


def run(bodies, ax, ay):
    steps_ramp = int(RAMP_S / DT)
    steps_hold = int(HOLD_S / DT)
    for i in range(steps_ramp + steps_hold):
        k = min(1.0, i / steps_ramp)
        # Accelerating the vehicle by +a = inertial force -a on the cargo.
        pb.setGravity(-ax * G * k, -ay * G * k, -G)
        pb.stepSimulation()
    out = []
    for p, b, pos0 in bodies:
        pos, orn = pb.getBasePositionAndOrientation(b)
        # Tilt: angle between the box's up axis and the world's.
        m = pb.getMatrixFromQuaternion(orn)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, m[8]))))
        shift = math.dist(pos, pos0) * 1000
        out.append((p, tilt, shift))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("request")
    ap.add_argument("result")
    ap.add_argument("--bin", type=int, default=None, help="only this bin (0-based)")
    ap.add_argument("--rear-bar", action="store_true", help="vehicle doors: rear face secured")
    ap.add_argument("--json", default=None)
    ap.add_argument("--rest", action="store_true", help="only a check without acceleration (the stand itself)")
    ap.add_argument("--dump", default=None, help="write tilt and shift of every box per test")
    ap.add_argument("--friction", type=float, default=None, help=f"friction coefficient (default {FRICTION})")
    a = ap.parse_args()
    if a.friction is not None:
        global BODY_FRICTION
        BODY_FRICTION = a.friction ** 0.5
    req = json.load(open(a.request))
    res = json.load(open(a.result))
    places = {p["id"]: p for p in req["available_packing_places"]}
    rule = req.get("pack_rule", {})
    lat = rule.get("accel_lateral_g", 0.5)
    lon = rule.get("accel_longitudinal_g", 0.8)
    pb.connect(pb.DIRECT)
    report = []
    total_bad = 0
    for bi, bin_ in enumerate(res["bins"]):
        if a.bin is not None and bi != a.bin:
            continue
        place = places[bin_["place_id"]]
        is_pallet = place["type"] == "PALLET"
        # Direction the cargo is thrown: (ax, ay) is the vehicle's acceleration; the load
        # moves the opposite way. Braking = vehicle accelerates towards +y (doors), cargo
        # goes to the front wall at y = 0.
        tests = [("покой (0 g)", 0, 0)] if a.rest else [
            ("влево (−X)", lat, 0), ("вправо (+X)", -lat, 0),
            ("вперёд (−Y)", 0, lat if is_pallet else lon), ("назад (+Y)", 0, -lat),
        ]
        bad = {}
        dump = {}
        for name, ax, ay in tests:
            bodies = build(place, bin_, a.rear_bar)
            for p, tilt, shift in run(bodies, ax, ay):
                dump.setdefault(name, {})[p["item_id"]] = (round(tilt, 1), round(shift))
                if tilt > MAX_TILT_DEG or shift > MAX_SHIFT_MM:
                    bad.setdefault(p["item_id"], []).append((name, round(tilt, 1), round(shift)))
        print(f"{bin_['bin_id']}: {len(bin_['placed_items'])} коробок, упало/сдвинулось: {len(bad)}")
        for iid, v in sorted(bad.items()):
            p = next(q for q in bin_["placed_items"] if q["item_id"] == iid)
            print(f"  {iid} ({p['width']}×{p['depth']}×{p['height']} @ {p['x']},{p['y']},{p['z']}): "
                  + "; ".join(f"{n}: наклон {t}°, сдвиг {s} мм" for n, t, s in v))
        total_bad += len(bad)
        report.append({"bin_id": bin_["bin_id"], "failed": [{"item_id": k, "tests": v} for k, v in bad.items()], "all": dump})
    if a.json:
        json.dump(report, open(a.json, "w"), ensure_ascii=False, indent=1)
    sys.exit(1 if total_bad else 0)


if __name__ == "__main__":
    main()
