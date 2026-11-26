"""Build blender/A240.blend - the detailed Aero A240 - from scratch.

    blender -b --factory-startup --python tools/blender/make_a240_blend.py

This is the generator that produced the .blend. Once you start editing A240.blend by
hand, treat the .blend as the source and do NOT re-run this script: it overwrites the
file. Export with tools/blender/export_a240.py (or tools/build_sdk.ps1).

All numbers are sim-frame metres (see a240_lib.py): +x right, +y up, +z nose, origin
at the wing quarter-chord on the propeller axis. The wheels, wing, eyepoint and cockpit
screen are where the .cfg files expect them. The exterior shape follows the AeroTrek
three-view (https://www.aerotrek.aero/photos/tech/dimensions.gif) and the side-on photos
on aerotrek.aero; the propeller plane, tail light, landing light and tail / nose scrape
points in engines.cfg, systems.cfg and flight_model.cfg match this geometry.
"""
import math
import os
import sys

import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a240_lib import (B, BLEND_PATH, FPS, MB, MODEL_DIR, SCREEN_MATERIAL, V, airfoil_loop,  # noqa: E402
                      airfoil_surface, boolean_apply, centroid, cutter_object, enable_msfs_exporter,
                      make_empty, make_object, rot_x, rot_y, rot_z, section)

# ============================================================================
# fuselage
# ============================================================================
GROUND_Y = -1.12

# z, half width, y bottom, y top, superellipse exponent
# Side profile from the AeroTrek three-view (aerotrek.aero/photos/tech/dimensions.gif)
# checked against side-on photos: a short, round cowl (spinner tip 1.51 m ahead of the
# wing leading edge) and a deep steel-tube tail cone whose sides and belly run in straight
# lines from the cabin back to the rudder post. The cabin keeps the published 44 in width.
FUSELAGE = [
    (1.60, 0.27, -0.17, 0.18, 2.4),     # cowl front, behind the spinner (small chin below it)
    (1.47, 0.36, -0.28, 0.23, 2.6),
    (1.25, 0.44, -0.38, 0.27, 2.8),
    (0.90, 0.51, -0.50, 0.30, 3.2),     # firewall / cowl joint
    (0.80, 0.53, -0.55, 0.34, 3.5),     # windshield base
    (0.31, 0.56, -0.58, 0.66, 4.0),     # windshield top / wing leading edge
    (0.00, 0.57, -0.58, 0.66, 4.0),
    (-0.45, 0.56, -0.56, 0.66, 4.0),
    (-0.94, 0.50, -0.50, 0.62, 3.6),    # aft cabin / wing trailing edge
    (-1.40, 0.415, -0.42, 0.47, 3.0),
    (-2.20, 0.265, -0.28, 0.33, 2.7),   # top dips where the dorsal fillet runs in...
    (-2.60, 0.19, -0.215, 0.31, 2.6),
    (-3.00, 0.12, -0.15, 0.35, 2.5),    # ...and rises again under the tailplane
    (-3.30, 0.065, -0.095, 0.38, 2.4),
    (-3.45, 0.035, -0.07, 0.385, 2.2),  # rudder post
]
FUSELAGE_BREAKS = [0.26, 0.31, -0.60, -0.80, 1.40, 0.90, 0.80]   # paint / glazing edges
RING_N = 64


def _pchip_slopes(xs, ys):
    """Fritsch-Carlson slopes: a smooth curve through the stations with no overshoot."""
    n = len(xs)
    h = [xs[i + 1] - xs[i] for i in range(n - 1)]
    d = [(ys[i + 1] - ys[i]) / h[i] for i in range(n - 1)]
    m = [d[0]] + [0.0] * (n - 2) + [d[-1]]
    for i in range(1, n - 1):
        if d[i - 1] * d[i] > 0:
            w1, w2 = 2 * h[i] + h[i - 1], h[i] + 2 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])
    return m


_FZ = [s[0] for s in reversed(FUSELAGE)]
_FCOLS = [[s[k] for s in reversed(FUSELAGE)] for k in range(1, 5)]
_FSLOPES = [_pchip_slopes(_FZ, col) for col in _FCOLS]


def fuselage_params(z):
    """(half width, y bottom, y top, exponent) at station z, smoothly interpolated."""
    if z >= _FZ[-1]:
        return tuple(col[-1] for col in _FCOLS)
    if z <= _FZ[0]:
        return tuple(col[0] for col in _FCOLS)
    i = max(k for k in range(len(_FZ) - 1) if _FZ[k] <= z)
    h = _FZ[i + 1] - _FZ[i]
    t = (z - _FZ[i]) / h
    h00, h10 = 2 * t ** 3 - 3 * t ** 2 + 1, t ** 3 - 2 * t ** 2 + t
    h01, h11 = -2 * t ** 3 + 3 * t ** 2, t ** 3 - t ** 2
    return tuple(h00 * col[i] + h10 * h * m[i] + h01 * col[i + 1] + h11 * h * m[i + 1]
                 for col, m in zip(_FCOLS, _FSLOPES))


def fus_point(z, phi_deg, scale=1.0):
    """Point on the skin at station z and polar angle phi (0 = right side, 90 = top)."""
    hw, yb, yt, e = fuselage_params(z)
    cy, hh = (yb + yt) / 2, (yt - yb) / 2
    phi = math.radians(phi_deg)
    c, s = math.cos(phi), math.sin(phi)
    r = (abs(c / hw) ** e + abs(s / hh) ** e) ** (-1.0 / e)
    return V(r * c * scale, cy + r * s * scale, z)


def fus_center(z):
    hw, yb, yt, e = fuselage_params(z)
    return V(0.0, (yb + yt) / 2, z)


def fus_out(p):
    """Approximate outward direction at a skin point."""
    return (V(p) - fus_center(p[2])).normalized()


def fuselage_ring(z, n=RING_N, scale=1.0):
    return [fus_point(z, 360.0 * j / n, scale) for j in range(n)]


def inner_halfwidth(z, y, scale=0.97):
    """Half width of the cabin at height y, a touch inside the skin."""
    hw, yb, yt, e = fuselage_params(z)
    cy, hh = (yb + yt) / 2, (yt - yb) / 2
    s = min(abs(y - cy) / hh, 0.999)
    return hw * (1 - s ** e) ** (1.0 / e) * scale


def fuselage_material(c):
    x, y, z = c
    hw, yb, yt, _ = fuselage_params(z)
    phi = math.degrees(math.atan2(y - (yb + yt) / 2, abs(x)))
    if 0.31 < z < 0.80 and phi > 45:
        return "glass"                                   # windshield
    if -0.80 < z < 0.26 and -22 < phi < 45:
        return "glass"                                   # door glazing
    if -0.60 < z < 0.26 and phi > 56:
        return "glass_tint"                              # skylight
    if z > 1.40:
        return "paint_red"                               # cowl nose
    if -34 < phi < -22:
        return "paint_red"                               # side stripe
    return "paint_white"


def fuselage_stations(step=0.05):
    keys = sorted(set([s[0] for s in FUSELAGE] + FUSELAGE_BREAKS), reverse=True)
    zs = []
    for a, b in zip(keys, keys[1:]):
        n = max(1, int(math.ceil((a - b) / step)))
        zs.extend(a + (b - a) * i / n for i in range(n))
    zs.append(keys[-1])
    return zs


def build_fuselage(col, parent):
    mb = MB()
    mb.loft([fuselage_ring(z) for z in fuselage_stations()], fuselage_material)
    obj = make_object("Fuselage", mb, col, parent, smooth=30)

    # cowling openings: the two round cooling inlets close beside the spinner and the
    # air exit under the cowl. Booleans leave dark pockets behind each opening.
    cut = MB()
    for side in (-1, 1):
        cut.tube((side * 0.19, 0.0, 1.75), (side * 0.19, 0.0, 1.45), 0.046, n=24, mat="inlet_dark")
    cut.rbox((0, -0.47, 1.00), (0.28, 0.10, 0.16), 0.02, "inlet_dark")
    boolean_apply(obj, [cutter_object(cut, col)])
    obj.data.set_sharp_from_angle(angle=math.radians(30))
    return obj


def build_liner(col, parent):
    """Cabin lining: the skin between the bulkheads, just inside it, facing inwards.
    Glazed panels are left open so the pilot sees out."""
    zs = [z for z in fuselage_stations() if -0.94 - 1e-6 <= z <= 0.95 + 1e-6]
    mb = MB()
    rings = [fuselage_ring(z, scale=0.992) for z in zs]
    idx = [[mb.v(p) for p in r] for r in rings]
    n = RING_N
    for a, b in zip(idx, idx[1:]):
        for j in range(n):
            k = (j + 1) % n
            q = [a[j], a[k], b[k], b[j]]
            c = centroid([mb.verts[i] for i in q])
            if fuselage_material(c).startswith("glass"):
                continue
            mb.f(q, "liner")
    return make_object("Cabin_Liner", mb, col, parent, smooth=30,
                       normals=lambda p: fus_center(p[2]) - p)


def panel_line(mb, pts, width=0.004, height=0.0012, mat="black", closed=False):
    """A thin dark strip lying on the skin along pts (skin points)."""
    outs = [fus_out(p) for p in pts]
    lifted = [V(p) + o * 0.0008 for p, o in zip(pts, outs)]
    prof = [(0, -width / 2), (height, -width / 2), (height, width / 2), (0, width / 2)]
    mb.sweep(lifted, mat=mat, profile=prof, ups=outs, closed=closed)


def build_fuselage_details(col, parent, fus_obj):
    mb = MB()
    # window seals along every glass / paint boundary of the skin
    seal_from_boundaries(mb)

    # cowling panel lines: the cowl / fuselage joint and the upper / lower cowl split
    ring = [fus_point(0.90, a) for a in range(0, 360, 4)]
    panel_line(mb, ring, closed=True)
    for side in (-1, 1):
        phi = 0 if side > 0 else 180
        line = [fus_point(z, phi) for z in [0.90 + i * 0.05 for i in range(14)]]
        panel_line(mb, line)
        # camloc fasteners along the split line
        for z in (0.97, 1.11, 1.25, 1.39, 1.52):
            p = fus_point(z, phi)
            o = fus_out(p)
            mb.disc(p + o * 0.0012, o, 0.007, 0.0025, n=10, mat="metal")
    # oil inspection door on top of the cowl
    door = ([fus_point(1.10, a) for a in range(78, 103, 3)] +
            [fus_point(z, 102) for z in (1.14, 1.18, 1.22, 1.26)] +
            [fus_point(1.30, a) for a in range(102, 77, -3)] +
            [fus_point(z, 78) for z in (1.26, 1.22, 1.18, 1.14)])
    panel_line(mb, door, closed=True)
    # gull-wing doors: outline, hinges along the top and the outside handle
    for side in (-1, 1):
        def dp(z, a):
            return fus_point(z, a if side > 0 else 180 - a)
        outline = ([dp(0.29, a) for a in range(-27, 53, 4)] +
                   [dp(z, 52) for z in [0.29 - i * 0.06 for i in range(20)]] +
                   [dp(-0.84, a) for a in range(52, -28, -4)] +
                   [dp(z, -27) for z in [-0.84 + i * 0.06 for i in range(19)]])
        panel_line(mb, outline, closed=True)
        for z in (0.12, -0.62):
            p = dp(z, 53.5)
            o = fus_out(p)
            mb.tube(p + o * 0.004 + V(0, 0, 0.035), p + o * 0.004 - V(0, 0, 0.035), 0.009, n=10, mat="metal")
        hp = dp(-0.70, -14)
        o = fus_out(hp)
        mb.sweep([hp + o * 0.002, hp + o * 0.022, hp + o * 0.022 - V(0, 0, 0.10)], radius=0.008,
                 n=8, mat="black")
    # fresh-air scoops either side of the windshield base
    for side in (-1, 1):
        p = fus_point(0.86, 8 if side > 0 else 172)
        o = fus_out(p)
        mb.ellipsoid(p + o * 0.004, 0.018, 0.022, 0.05, "black", nu=6, nv=10)
    # exhaust: tailpipe out of the cowl exit, slightly to the right
    mb.sweep([(0.08, -0.42, 1.05), (0.09, -0.50, 1.01), (0.10, -0.555, 0.94)], radius=0.021, n=14,
             mat="metal_dark")
    mb.sweep([(-0.08, -0.43, 1.06), (-0.08, -0.52, 1.03)], radius=0.016, n=12, mat="metal_dark")
    # LED landing / taxi lights in the cowl front, under the spinner
    for side in (-1, 1):
        mb.ellipsoid((side * 0.065, -0.145, 1.600), 0.022, 0.016, 0.012, "lens_clear", nu=6, nv=14)
    # antennas: COM whip on the top, transponder stub under the belly, GPS puck
    top = fus_point(-1.25, 90)
    mb.disc(top + V(0, 0.003, 0), (0, 1, 0), 0.025, 0.006, n=14, mat="black")
    mb.tube(top, top + V(0, 0.52, -0.30), 0.0045, 0.002, n=6, mat="black")
    belly = fus_point(-0.55, 270)
    mb.disc(belly - V(0, 0.003, 0), (0, 1, 0), 0.018, 0.006, n=12, mat="black")
    mb.tube(belly, belly - V(0, 0.075, 0), 0.0035, n=6, mat="black")
    mb.ellipsoid(belly - V(0, 0.078, 0), 0.006, 0.006, 0.006, "black", nu=4, nv=8)
    gps = fus_point(-1.60, 90)
    mb.ellipsoid(gps + V(0, 0.004, 0), 0.04, 0.012, 0.05, "paint_white", nu=6, nv=14)
    # tail tie-down ring and tail skid
    tail = fus_point(-3.05, 270)
    mb.sweep([tail + V(0.0, -0.01 - 0.018 * math.sin(a), 0.018 * math.cos(a))
              for a in [2 * math.pi * j / 16 for j in range(16)]], radius=0.004, n=6, mat="metal",
             closed=True)
    # spring tail skid curling down and aft from under the rudder post
    s0 = fus_point(-3.28, 270) + V(0, 0.01, 0)
    mb.sweep([s0, s0 + V(0, -0.06, -0.07), s0 + V(0, -0.105, -0.17), s0 + V(0, -0.115, -0.23),
              s0 + V(0, -0.105, -0.26)], radius=0.007, n=8, mat="metal")
    mb.rbox(s0 + V(0, -0.004, 0.01), (0.03, 0.02, 0.06), 0.005, "metal")
    # static vent buttons
    for side in (-1, 1):
        p = fus_point(-1.75, 0 if side > 0 else 180)
        mb.disc(p, fus_out(p), 0.006, 0.003, n=10, mat="metal")
    return make_object("Fuselage_Details", mb, col, parent, smooth=40)


def seal_from_boundaries(mb):
    """Rubber seals along the edges between glazing and paint on the skin grid."""
    zs = fuselage_stations()
    n = RING_N
    grid = [fuselage_ring(z) for z in zs]
    mats = {}
    for i in range(len(zs) - 1):
        for j in range(n):
            k = (j + 1) % n
            c = centroid([grid[i][j], grid[i][k], grid[i + 1][k], grid[i + 1][j]])
            mats[(i, j)] = fuselage_material(c)
    edges = set()

    def key(a, b):
        return (a, b) if a <= b else (b, a)
    for (i, j), m in mats.items():
        # neighbour across the edge at constant z (i+1 side) and across constant j
        for (ii, jj), edge in (((i + 1, j), ((i + 1, j), (i + 1, (j + 1) % n))),
                               ((i, (j + 1) % n), ((i, (j + 1) % n), (i + 1, (j + 1) % n)))):
            if (ii, jj) not in mats:
                continue
            other = mats[(ii, jj)]
            if (m.startswith("glass")) != (other.startswith("glass")) or \
               (m.startswith("glass") and other.startswith("glass") and m != other):
                edges.add(key(*edge))
    # chain edges into polylines
    adj = {}
    for a, b in edges:
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)
    used = set()
    for start in list(adj):
        for nxt in adj[start]:
            if key(start, nxt) in used:
                continue
            path = [start, nxt]
            used.add(key(start, nxt))
            while True:
                cur = path[-1]
                if len(adj[cur]) != 2:
                    break
                cand = [x for x in adj[cur] if key(cur, x) not in used]
                if not cand:
                    break
                used.add(key(cur, cand[0]))
                path.append(cand[0])
            closed = path[0] == path[-1]
            pts = [grid[i][j] for i, j in (path[:-1] if closed else path)]
            outs = [fus_out(p) for p in pts]
            pts = [V(p) + o * 0.003 for p, o in zip(pts, outs)]
            if len(pts) >= 2:
                mb.sweep(pts, radius=0.0065, n=6, mat="rubber", closed=closed, ups=outs, flat=0.6)


# ============================================================================
# wing
# ============================================================================
WING_Y = 0.60
WING_LE_Z = 0.31
WING_CHORD = 1.02
WING_INCIDENCE = math.radians(2.0)
WING_AIRFOIL = (0.04, 0.4, 0.13)
WING_TIP_X = 4.565          # 29 ft 11.3 in span
WING_PAINT_X = 3.95         # paint boundary of the coloured tip
FLAPERON_PIVOT = (0.0, 0.50, -0.72)
FLAPERON_HANGERS = (0.70, 1.55, 2.40, 3.27, 4.15)
WCD = V(0.0, -math.sin(WING_INCIDENCE), -math.cos(WING_INCIDENCE))   # chord direction (LE -> TE)
WTD = V(0.0, math.cos(WING_INCIDENCE), -math.sin(WING_INCIDENCE))    # thickness direction


def wing_point(side, x, xc, upper=True, lift=0.0):
    """Point on the constant-chord wing panel (|x| <= WING_PAINT_X) and its outward normal."""
    yc, (nx, ny) = airfoil_surface(*WING_AIRFOIL, xc, upper)
    o = V(side * x, WING_Y, WING_LE_Z)
    nrm = (WCD * nx + WTD * ny).normalized()
    return o + WCD * (xc * WING_CHORD) + WTD * (yc * WING_CHORD) + nrm * lift, nrm


def roof_y(x, z):
    """Height of the cabin roof skin at lateral position x, station z."""
    hw, yb, yt, e = fuselage_params(z)
    cy, hh = (yb + yt) / 2, (yt - yb) / 2
    u = min(abs(x) / hw, 0.999)
    return cy + hh * (1 - u ** e) ** (1.0 / e)


def wing_tip_stations():
    # (x, chord scale, LE set-back, thickness scale): straight panel, then the moulded tip
    # cap. The real tip is square in plan with only its edges rounded off.
    t = WING_TIP_X
    st = [(0.50, 1, 0, 1), (0.80, 1, 0, 1), (1.4, 1, 0, 1), (2.0, 1, 0, 1), (2.55, 1, 0, 1),
          (3.1, 1, 0, 1), (3.6, 1, 0, 1), (WING_PAINT_X, 1, 0, 1), (t - 0.17, 1, 0, 1),
          (t - 0.07, 0.995, 0.002, 0.97), (t - 0.035, 0.985, 0.007, 0.90), (t - 0.016, 0.965, 0.016, 0.76),
          (t - 0.006, 0.935, 0.032, 0.55), (t, 0.89, 0.06, 0.25)]
    return st


def build_wing(side, col, parent):
    mb = MB()
    m, p, t = WING_AIRFOIL
    rings = []
    for x, cs, setback, ts in wing_tip_stations():
        loop = airfoil_loop(m, p, t * ts, n=24)
        chord = WING_CHORD * cs
        rings.append(section((side * x, WING_Y, WING_LE_Z - setback), chord, WCD, WTD, loop))
    # root fairing: the root section carried inboard over the cabin roof, its lower
    # surface laid onto the roof, so there is no gap between wing and cabin
    root = rings[0]
    fairing = []
    for x in (0.38, 0.44):
        ring = []
        for rp in root:
            q = V(side * x, rp.y, rp.z)
            q.y = max(q.y, roof_y(x, q.z) + 0.004)
            ring.append(q)
        fairing.append(ring)
    rings = fairing + rings
    mb.loft(rings, lambda c: "paint_red" if abs(c[0]) > WING_PAINT_X else "paint_white")

    # rib tapes: the fabric wing shows every rib as a slight ridge
    loop = airfoil_loop(m, p, t, 0.0, 1.0, n=24)
    for x in [0.75 + 0.30 * i for i in range(12)]:
        pts, ups = [], []
        for xc, yc in loop[1:-1]:
            if xc < 0.015 or xc > 0.97:
                continue
            upper = yc > (m / p ** 2 * (2 * p * xc - xc * xc) if xc < p else
                          m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * xc - xc * xc))
            pt, nrm = wing_point(side, x, xc, upper, lift=0.0006)
            pts.append(pt)
            ups.append(nrm)
        # split into the upper and lower run (skip leading edge, the tape wraps it on the real wing)
        prof = [(0, -0.012), (0.0012, -0.009), (0.0012, 0.009), (0, 0.012)]
        mb.sweep(pts, mat="paint_white" if x < WING_PAINT_X else "paint_red", profile=prof, ups=ups, cap=True)

    # fuel cap over the root tank
    pt, nrm = wing_point(side, 0.82, 0.22, True)
    mb.disc(pt + nrm * 0.004, nrm, 0.036, 0.008, n=20, mat="metal")
    mb.rbox(pt + nrm * 0.010, (0.05, 0.010, 0.012), 0.003, "metal",
            rot=Matrix.Rotation(math.atan2(nrm.z, nrm.y), 3, "X"))
    # fold fittings at the rear spar root, strut fittings under the wing
    for xc in (0.62,):
        pt, nrm = wing_point(side, 0.47, xc, False)
        mb.rbox(pt - V(0, 0.012, 0), (0.06, 0.03, 0.05), 0.006, "metal")
    for xc in (0.24, 0.80):
        pt, nrm = wing_point(side, 2.55, xc, False)
        mb.rbox(pt - V(0, 0.010, 0), (0.05, 0.03, 0.06), 0.006, "metal")
    # flaperon hangers: plates from the wing underside down to the hinge line
    for x in FLAPERON_HANGERS:
        top_f, _ = wing_point(side, x, 0.78, False)
        top_r, _ = wing_point(side, x, 0.93, False)
        hinge = V(side * x, FLAPERON_PIVOT[1], FLAPERON_PIVOT[2])
        prof = [(-0.005, -0.0), (0.005, 0.0)]
        plate = MB()
        rings = []
        for a in (-0.005, 0.005):
            off = V(a, 0, 0)
            rings.append([top_f + V(0, 0.01, 0) + off, top_r + V(0, 0.01, 0) + off,
                          hinge + V(0, 0.0, -0.02) + off, hinge + V(0, 0.02, 0.025) + off])
        plate.loft(rings, "paint_white")
        mb.extend(plate)
        mb.tube(hinge + V(-0.012, 0, 0), hinge + V(0.012, 0, 0), 0.008, n=10, mat="metal")
    if side < 0:
        # pitot tube under the left wing, stall warning vane on the leading edge
        pt, nrm = wing_point(side, 2.65, 0.28, False)
        mast = [pt + V(0, 0.01, 0), pt - V(0, 0.09, 0), pt - V(0, 0.11, -0.02), pt - V(0, 0.11, -0.26)]
        mb.sweep(mast, radius=0.006, n=10, mat="metal")
        mb.rbox(pt - V(0, 0.04, 0), (0.008, 0.10, 0.05), 0.003, "metal")
        le, _ = wing_point(side, 1.55, 0.012, False)
        mb.rbox(le + V(0, -0.006, 0.012), (0.02, 0.003, 0.025), 0.001, "metal")
    # navigation (red left / green right) and strobe lenses on the tip
    tip_x = side * (WING_TIP_X - 0.003)
    mb.ellipsoid((tip_x, WING_Y + 0.02, -0.12), 0.016, 0.024, 0.055, "lens_red" if side < 0 else "lens_green",
                 nu=6, nv=12)
    mb.ellipsoid((tip_x, WING_Y + 0.02, -0.24), 0.014, 0.020, 0.045, "lens_clear", nu=6, nv=12)
    return make_object("Wing_%s" % ("L" if side < 0 else "R"), mb, col, parent, smooth=34)


def build_flaperon(side, col, parent, pivot):
    mb = MB()
    loop = airfoil_loop(0.0, 0.0, 0.14, n=12)
    end = WING_TIP_X - 0.175
    xs = [0.62, 0.66, 1.2, 2.0, 2.8, 3.6, end - 0.26, end - 0.04, end]
    rings = []
    for i, x in enumerate(xs):
        th = 0.6 if i in (0, len(xs) - 1) else 1.0          # rounded end caps
        l2 = airfoil_loop(0.0, 0.0, 0.14 * th, n=12)
        c = 0.23 if th == 1.0 else 0.21
        rings.append(section((side * x, 0.50, -0.70 - (0.23 - c) / 2), c, (0, 0, -1), (0, 1, 0), l2))
    mb.loft(rings, lambda c: "paint_red" if abs(c[0]) > WING_PAINT_X else "paint_white")
    # rib tapes
    for x in (1.0, 1.6, 2.2, 2.8, 3.4, 4.0):
        pts, ups = [], []
        for xc, yc in loop[1:-1]:
            if 0.03 < xc < 0.96:
                p0 = V(side * x, 0.50 + yc * 0.23, -0.70 - xc * 0.23)
                n0 = V(0, 1 if yc > 0 else -1, 0)
                pts.append(p0 + n0 * 0.0005)
                ups.append(n0)
        mb.sweep(pts, mat="paint_white" if x < WING_PAINT_X else "paint_red",
                 profile=[(0, -0.01), (0.001, -0.008), (0.001, 0.008), (0, 0.01)], ups=ups)
    # hinge horns under the leading edge at each hanger
    for x in FLAPERON_HANGERS:
        mb.rbox((side * x + side * 0.016, 0.50, -0.715), (0.008, 0.03, 0.05), 0.003, "metal")
    return make_object("SKEL_Aileron_%s" % ("L" if side < 0 else "R"), mb, col, parent, pivot=pivot, smooth=34)


# Both wing struts and the main gear leg meet at one fitting on the lower longeron at the
# rear of the cabin (the three-view and side photos agree on this station).
STRUT_FOOT = (0.55, -0.50, -0.40)


def build_struts(side, col, parent):
    mb = MB()
    foot = V(side * STRUT_FOOT[0], STRUT_FOOT[1], STRUT_FOOT[2])
    tops = [V(side * 2.55, 0.555, 0.06), V(side * 2.55, 0.560, -0.52)]
    loop = airfoil_loop(0, 0, 0.36, n=8)
    prof = [((0.35 - xc) * 0.075, yc * 0.075) for xc, yc in loop]   # streamlined, 75 mm chord
    for top in tops:
        a = foot + (top - foot).normalized() * 0.06
        b = top - (top - foot).normalized() * 0.05
        mb.sweep([a, b], mat="paint_white", profile=prof, up=(0, 0, 1))
        # end fittings
        for p, d in ((a, (top - foot).normalized()), (b, (foot - top).normalized())):
            mb.tube(p, p - d * 0.06, 0.011, 0.008, n=10, mat="metal")
            mb.tube(p - d * 0.055 + V(0, 0, 0.012), p - d * 0.055 - V(0, 0, 0.012), 0.006, n=8, mat="metal")
    # root fitting on the fuselage
    mb.rbox(foot + V(-side * 0.01, 0, 0), (0.04, 0.06, 0.10), 0.01, "metal")
    # jury struts from each main strut up to the wing
    for top, wz in zip(tops, (-0.02, -0.38)):
        t = (1.75 - STRUT_FOOT[0]) / (2.55 - STRUT_FOOT[0])
        p = foot + (top - foot) * t
        w, _ = wing_point(side, 1.75, (WING_LE_Z - wz) / WING_CHORD, False)
        mb.tube(p, w + V(0, 0.004, 0), 0.010, n=8, mat="paint_white")
    return make_object("Struts_%s" % ("L" if side < 0 else "R"), mb, col, parent, smooth=40)


# ============================================================================
# landing gear
# ============================================================================
MAIN_X, MAIN_Z, MAIN_R = 1.00, -0.35, 0.19
NOSE_Z, NOSE_R = 1.25, 0.165


def wheel(mb, c, radius, width, with_brake=True, side=1):
    """Tyre (axle along x) with tread grooves, rim, hub cap and disc brake."""
    c = V(c)
    n = 40
    prof = [(-0.50, 0.56), (-0.48, 0.74), (-0.42, 0.88), (-0.30, 0.965), (-0.16, 1.0), (-0.12, 0.985),
            (-0.08, 1.0), (0.08, 1.0), (0.12, 0.985), (0.16, 1.0), (0.30, 0.965), (0.42, 0.88),
            (0.48, 0.74), (0.50, 0.56)]
    rings = []
    for fx, fr in prof:
        rings.append([c + V(fx * width, radius * fr * math.cos(2 * math.pi * j / n),
                            radius * fr * math.sin(2 * math.pi * j / n)) for j in range(n)])
    mb.loft(rings, "tire", cap0=False, cap1=False)
    # rims close the tyre on both faces
    for s in (-1, 1):
        rim = [c + V(s * 0.5 * width, radius * 0.56 * math.cos(2 * math.pi * j / n),
                     radius * 0.56 * math.sin(2 * math.pi * j / n)) for j in range(n)]
        inner = [c + V(s * 0.40 * width, radius * 0.50 * math.cos(2 * math.pi * j / n),
                       radius * 0.50 * math.sin(2 * math.pi * j / n)) for j in range(n)]
        hub = [c + V(s * 0.42 * width, radius * 0.22 * math.cos(2 * math.pi * j / n),
                     radius * 0.22 * math.sin(2 * math.pi * j / n)) for j in range(n)]
        cap = [c + V(s * 0.55 * width, radius * 0.12 * math.cos(2 * math.pi * j / n),
                     radius * 0.12 * math.sin(2 * math.pi * j / n)) for j in range(n)]
        r = [rim, inner, hub, cap] if s > 0 else [cap, hub, inner, rim]
        mb.loft(r, "metal", cap0=(s < 0), cap1=(s > 0))
        for b in range(5):   # wheel bolts
            a = 2 * math.pi * b / 5
            p = c + V(s * 0.43 * width, radius * 0.30 * math.cos(a), radius * 0.30 * math.sin(a))
            mb.disc(p, (1, 0, 0), 0.006, 0.01, n=6, mat="chrome")
    if with_brake:
        s = -side                                            # brake on the inboard face
        d = c + V(s * (0.5 * width + 0.012), 0, 0)
        mb.disc(d, (1, 0, 0), radius * 0.62, 0.005, n=32, mat="metal")
        mb.rbox(d + V(s * 0.004, radius * 0.48, -radius * 0.25), (0.035, 0.06, 0.05), 0.008, "metal_dark",
                rot=rot_x(-25))


def wheel_pant(c, rx, ry, rz, n=40):
    """A streamlined spat around the wheel centre c (a closed solid)."""
    c = V(c)
    rings = []
    # cosine spacing puts more rings into the rounded nose and the tail
    stations = [-math.cos(math.pi * i / 23) for i in range(24)]
    for t in stations:
        # front (t>0) an ellipse, tail (t<0) a longer, finer taper; both flat at t=0
        if t >= 0:
            f = math.sqrt(max(1 - t * t, 0.0))
        else:
            f = max(1 - t * t, 0.0) ** 0.65
        f = max(f, 0.03)
        zz = c.z + rz * t
        cy = c.y + (0.03 * (1 - f) if t < 0 else 0.0)    # tail sweeps up slightly
        ring = []
        for j in range(n):
            a = 2 * math.pi * j / n
            ca, sa = math.cos(a), math.sin(a)
            e = 2.4
            r = (abs(ca) ** e + abs(sa) ** e) ** (-1 / e)
            ring.append(V(c.x + rx * f * r * ca, cy + ry * f * r * sa, zz))
        rings.append(ring)
    return rings


def build_gear(col, parent):
    objs = []
    for side in (-1, 1):
        mb = MB()
        axle = V(side * MAIN_X, GROUND_Y + MAIN_R, MAIN_Z)
        # straight composite leg from the strut fitting, splayed out and down to the axle
        top = V(side * (STRUT_FOOT[0] - 0.04), STRUT_FOOT[1] - 0.05, STRUT_FOOT[2])
        low = V(side * (MAIN_X - 0.085), axle.y + 0.03, MAIN_Z)
        mb.tube(top, low, 0.026, 0.021, n=14, mat="paint_white")
        mb.rbox(top + V(side * 0.01, 0.01, 0), (0.06, 0.05, 0.09), 0.008, "metal")        # leg socket
        mb.rbox(axle + V(-side * 0.09, 0.01, 0), (0.03, 0.07, 0.08), 0.008, "metal")    # axle plate
        wheel(mb, axle, MAIN_R, 0.13, side=side)
        # step on the leg for getting in
        sp = top + (low - top) * 0.30
        mb.tube(sp, sp + V(0, 0.0, 0.16), 0.009, n=8, mat="metal")
        mb.rbox(sp + V(0, 0.008, 0.18), (0.07, 0.008, 0.06), 0.004, "black")
        # wheel spat with a slot for the tyre
        pant = MB()
        pant.loft(wheel_pant(axle + V(0, 0.045, -0.03), 0.115, 0.19, 0.38), "paint_red")
        pobj = make_object("Wheelpant_%s" % ("L" if side < 0 else "R"), pant, col, parent, smooth=60)
        slot = MB()
        slot.rbox(axle + V(0, -0.17, -0.03), (0.155, 0.22, 0.50), 0.04, "inlet_dark")
        boolean_apply(pobj, [cutter_object(slot, col)])
        pobj.data.set_sharp_from_angle(angle=math.radians(60))
        objs.append(pobj)
        objs.append(make_object("Gear_%s" % ("L" if side < 0 else "R"), mb, col, parent, smooth=40))

    # nose gear: a long leg raked well forward from under the cabin floor to the fork, held
    # up by a sprung shock strut from the bottom of the firewall; fork, wheel and spat
    mb = MB()
    axle = V(0.0, GROUND_Y + NOSE_R, NOSE_Z)
    top = V(0.0, -0.535, 0.72)
    mid = V(0.0, -0.80, NOSE_Z - 0.09)
    mb.tube(top, mid, 0.024, 0.022, n=14, mat="paint_white")
    mb.rbox(top + V(0, 0.01, 0), (0.09, 0.04, 0.07), 0.008, "metal")                    # pivot
    leg = (mid - top).normalized()
    shock_lo = top + leg * ((0.93 - top.z) / leg.z)
    shock_hi = V(0.0, -0.475, 0.93)
    mb.tube(shock_hi, shock_lo, 0.017, n=12, mat="metal_dark")
    for i in range(5):                                                                  # spring
        t = 0.2 + 0.15 * i
        c = shock_hi + (shock_lo - shock_hi) * t
        mb.sweep([c + V(0.026 * math.cos(a), 0, 0.026 * math.sin(a))
                  for a in [2 * math.pi * j / 16 for j in range(16)]],
                 radius=0.006, n=6, mat="metal", closed=True)
    mb.rbox(mid + V(0, -0.02, 0), (0.17, 0.035, 0.07), 0.01, "metal")                   # fork crown
    for s in (-1, 1):
        mb.sweep([mid + V(s * 0.072, -0.02, 0), axle + V(s * 0.072, 0, 0)], mat="metal", up=(0, 0, 1),
                 profile=[(-0.025, -0.006), (0.025, -0.006), (0.025, 0.006), (-0.025, 0.006)])
    wheel(mb, axle, NOSE_R, 0.11, with_brake=False)
    mb.tube(axle + V(-0.085, 0, 0), axle + V(0.085, 0, 0), 0.012, n=10, mat="metal")
    objs.append(make_object("Gear_Nose", mb, col, parent, smooth=40))
    pant = MB()
    pant.loft(wheel_pant(axle + V(0, 0.04, -0.02), 0.10, 0.17, 0.32), "paint_red")
    pobj = make_object("Wheelpant_Nose", pant, col, parent, smooth=60)
    slot = MB()
    slot.rbox(axle + V(0, -0.15, -0.02), (0.13, 0.20, 0.42), 0.035, "inlet_dark")
    slot.rbox(axle + V(0, 0.16, -0.0), (0.19, 0.12, 0.14), 0.02, "inlet_dark")        # fork opening
    boolean_apply(pobj, [cutter_object(slot, col)])
    pobj.data.set_sharp_from_angle(angle=math.radians(60))
    objs.append(pobj)
    return objs


# ============================================================================
# tail
# ============================================================================
# The tailplane sits on top of the tail cone at the foot of the fin, braced by a V strut
# each side; swept leading edge, straight hinge and trailing edge, 7 ft 10 in span (the
# folded width). The fin has a straight swept leading edge running into a concave dorsal
# fillet, a flat top and a vertical trailing edge; the rudder runs down past the tail cone.
STAB_Y = 0.43
ELEV_HINGE_Z = -3.32
RUDDER_HINGE_Z = -3.48
RUDDER_TE_Z = -3.76
FIN_TOP_Y = 1.13
TAIL_T = 0.09
FIN_T = 0.08


def _round(u):
    u = min(max(u, 0.0), 1.0)
    return 1 - math.sqrt(max(1 - u * u, 0.0))


def stab_planform(x):
    """(LE z, TE z) of the horizontal tail at span x, with rounded tips."""
    ax = abs(x)
    le = -2.82 - 0.24 * min(ax, 1.08) / 1.08
    te = -3.66 + 0.04 * min(ax, 1.08) / 1.08
    if ax > 1.08:
        r = _round((ax - 1.08) / 0.115)
        le -= 0.12 * r
        te += 0.10 * r
    return le, te


STAB_X = [0.0, 0.16, 0.4, 0.8, 1.08, 1.13, 1.16, 1.18, 1.19, 1.195]


def tail_section_pts(x, le, te, hinge, part, n, t=TAIL_T, y=STAB_Y, vertical=False):
    chord = le - te
    f = min(max((le - hinge) / chord, 0.02), 0.98)
    loop = airfoil_loop(0, 0, t, 0.0, f, n=n) if part == "fixed" else airfoil_loop(0, 0, t, f, 1.0, n=n)
    if vertical:
        return section((0, x, le), chord, (0, 0, -1), (1, 0, 0), loop)
    return section((x, y, le), chord, (0, 0, -1), (0, 1, 0), loop)


def build_stabilizer(col, parent):
    mb = MB()
    xs = [-x for x in reversed(STAB_X[1:])] + STAB_X
    rings = []
    for x in xs:
        le, te = stab_planform(x)
        rings.append(tail_section_pts(x, le, te, ELEV_HINGE_Z, "fixed", 10))
    mb.loft(rings, "paint_white")
    for x in (-0.95, -0.55, -0.28, 0.28, 0.55, 0.95):
        le, te = stab_planform(x)
        chord = le - te
        f = (le - ELEV_HINGE_Z) / chord
        for upper in (True, False):
            pts, ups = [], []
            for i in range(1, 9):
                xc = f * i / 9.0
                yc, (nx, ny) = airfoil_surface(0, 0, TAIL_T, xc, upper)
                pts.append(V(x, STAB_Y + yc * chord + ny * 0.0005, le - xc * chord))
                ups.append(V(0, ny, -nx))
            mb.sweep(pts, mat="paint_white", ups=ups,
                     profile=[(0, -0.009), (0.001, -0.007), (0.001, 0.007), (0, 0.009)])
    # V struts from the lower tail cone up to the underside of each half
    loop = airfoil_loop(0, 0, 0.36, n=8)
    prof = [((0.35 - xc) * 0.035, yc * 0.035) for xc, yc in loop]
    for side in (-1, 1):
        foot = V(side * 0.115, 0.21, -2.93)
        for z in (-2.95, -3.24):
            le, te = stab_planform(0.55)
            yc, _ = airfoil_surface(0, 0, TAIL_T, (le - z) / (le - te), False)
            top = V(side * 0.55, STAB_Y + yc * (le - te) + 0.004, z)
            mb.sweep([foot, top], mat="paint_white", profile=prof, up=(0, 0, 1))
            mb.tube(top + V(0, -0.012, 0), top + V(0, 0.003, 0), 0.008, n=8, mat="metal")
        mb.rbox(foot, (0.02, 0.035, 0.05), 0.006, "metal")
    return make_object("Stabilizer", mb, col, parent, smooth=34)


def build_elevator(col, parent, pivot):
    mb = MB()
    for side in (-1, 1):
        xs = [side * x for x in STAB_X[1:]]
        rings = []
        for x in xs:
            le, te = stab_planform(x)
            rings.append(tail_section_pts(x, le, te, ELEV_HINGE_Z, "move", 8))
        mb.loft(rings, "paint_white")
        # control horn under the root
        mb.rbox((side * 0.16, STAB_Y - 0.05, ELEV_HINGE_Z + 0.01), (0.008, 0.09, 0.05), 0.003, "metal")
    # trim tab on the left elevator: outline and actuator horn
    x0, x1 = -0.22, -0.62
    for upper in (True, False):
        pts = []
        for x, xc in ((x0, 0.86), (x0, 0.995), (x1, 0.995), (x1, 0.86), (x0, 0.86)):
            le, te = stab_planform(x)
            chord = le - te
            yc, (nx, ny) = airfoil_surface(0, 0, TAIL_T, min(xc, 0.99), upper)
            pts.append(V(x, STAB_Y + yc * chord + ny * 0.0012, le - xc * chord))
        for a, b in zip(pts, pts[1:]):
            mb.sweep([a, b], radius=0.0018, n=4, mat="black", up=(0, 1, 0))
    mb.rbox(((x0 + x1) / 2, STAB_Y + 0.02, -3.40), (0.006, 0.03, 0.04), 0.002, "metal")
    return make_object("SKEL_Elevator", mb, col, parent, pivot=pivot, smooth=34)


FIN_Y = [0.27, 0.32, 0.37, 0.40, 0.43, 0.47, 0.51, 0.55, 0.70, 0.85, 1.00, 1.06, 1.10, 1.12, 1.13]
RUDDER_Y = [-0.07, -0.05, -0.02, 0.02, 0.06, 0.15, 0.25] + FIN_Y


def fin_planform(y):
    """(LE z, rudder TE z) of the fin and rudder at height y."""
    if y >= 0.55:
        le = -2.80 - 0.42 * (y - 0.55) / (FIN_TOP_Y - 0.55)
    elif y >= 0.40:
        le = -2.80 + 0.40 * ((0.55 - y) / 0.15) ** 2         # concave dorsal fillet
    else:
        le = -2.40 + 3.0 * (0.40 - y)                         # buried in the tail cone
    te = RUDDER_TE_Z
    if y > 1.05:                                              # rounded top corners
        r = _round((y - 1.05) / (FIN_TOP_Y - 1.05))
        le -= 0.12 * r
        te += 0.04 * r
    if y < 0.06:                                              # rounded rudder heel
        te += 0.16 * _round((0.06 - y) / 0.13)
    return le, te


def fin_thickness(le, te):
    """Keep the long fillet sections thin: no more than ~68 mm thick."""
    return FIN_T * min(1.0, 0.85 / (le - te))


def build_fin(col, parent):
    mb = MB()
    rings = []
    for y in FIN_Y:
        le, te = fin_planform(y)
        rings.append(tail_section_pts(y, le, te, RUDDER_HINGE_Z, "fixed", 10, t=fin_thickness(le, te),
                                      vertical=True))
    mb.loft(rings, lambda c: "paint_red" if c[1] > 0.80 else "paint_white")
    for y in (0.62, 0.80, 0.98):
        le, te = fin_planform(y)
        chord = le - te
        t = fin_thickness(le, te)
        f = (le - RUDDER_HINGE_Z) / chord
        for upper in (True, False):
            pts, ups = [], []
            for i in range(1, 9):
                xc = f * i / 9.0
                yc, (nx, ny) = airfoil_surface(0, 0, t, xc, upper)
                pts.append(V(yc * chord + ny * 0.0005, y, le - xc * chord))
                ups.append(V(ny, 0, -nx))
            mb.sweep(pts, mat="paint_red" if y > 0.80 else "paint_white", ups=ups,
                     profile=[(0, -0.009), (0.001, -0.007), (0.001, 0.007), (0, 0.009)])
    return make_object("Fin", mb, col, parent, smooth=34)


def build_rudder(col, parent, pivot):
    mb = MB()
    rings = []
    root_le, root_te = fin_planform(FIN_Y[0])
    for y in RUDDER_Y:
        le, te = fin_planform(y)
        if y < FIN_Y[0]:
            le = root_le                     # below the fin: same section as at its root
            t = fin_thickness(root_le, root_te)
        else:
            t = fin_thickness(le, te)        # matches the fin at the hinge
        rings.append(tail_section_pts(y, le, te, RUDDER_HINGE_Z, "move", 8, t=t, vertical=True))
    mb.loft(rings, lambda c: "paint_red" if c[1] > 0.80 else "paint_white")
    # tail navigation light on the trailing edge, and the control horns at the bottom
    mb.ellipsoid((0, 0.30, RUDDER_TE_Z + 0.005), 0.012, 0.03, 0.018, "lens_clear", nu=6, nv=10)
    for side in (-1, 1):
        mb.rbox((side * 0.035, 0.0, RUDDER_HINGE_Z - 0.02), (0.05, 0.035, 0.008), 0.003, "metal")
    return make_object("SKEL_Rudder", mb, col, parent, pivot=pivot, smooth=34)


# ============================================================================
# propeller
# ============================================================================
PROP_Z = 1.64
PROP_R = 0.79         # 5.18 ft (62 in) diameter, from the AeroTrek three-view
SPINNER_Z0, SPINNER_LEN, SPINNER_R = 1.615, 0.255, 0.125


def build_spinner(col, parent):
    mb = MB()
    # a long, pointed spinner (an ogive rather than the old half-dome)
    z0, length, r0 = SPINNER_Z0, SPINNER_LEN, SPINNER_R
    rings = []
    for t in (0.0, 0.08, 0.18, 0.3, 0.42, 0.54, 0.65, 0.75, 0.83, 0.9, 0.95, 0.985):
        r = r0 * (1 - t ** 1.8) ** 0.72
        rings.append([V(r * math.cos(2 * math.pi * j / 48), r * math.sin(2 * math.pi * j / 48), z0 + length * t)
                      for j in range(48)])
    mb.loft(rings, "paint_red")
    mb.disc((0, 0, z0 - 0.005), (0, 0, 1), r0 + 0.003, 0.01, n=48, mat="metal")    # backplate
    return make_object("Spinner", mb, col, parent, smooth=40)


def build_blades(col, parent, pivot):
    mb = MB()
    stations = [(0.07, 0.050, 60, "round"), (0.12, 0.070, 55, "mix"), (0.17, 0.095, 47, None),
                (0.24, 0.108, 40, None), (0.33, 0.115, 33, None), (0.43, 0.113, 28, None),
                (0.53, 0.106, 24, None), (0.63, 0.097, 21, None), (0.72, 0.086, 19, None),
                (0.79, 0.074, 17.5, None), (0.83, 0.062, 16.8, None), (0.855, 0.046, 16.2, None),
                (0.864, 0.024, 16, None)]
    k = PROP_R / 0.864        # stations were laid out for a 68 in blade; scale to PROP_R
    stations = [(r * k if r > 0.10 else r, c * (0.6 + 0.4 * k), b, kind) for r, c, b, kind in stations]
    loop = airfoil_loop(0.02, 0.4, 0.11, n=10)
    circ = [(0.4 + 0.4 * math.cos(math.atan2(yc, xc - 0.4)), 0.4 * math.sin(math.atan2(yc, xc - 0.4)))
            for xc, yc in loop]
    for b in range(3):
        psi = math.radians(90 + 120 * b)
        radial = V(math.cos(psi), math.sin(psi), 0.0)
        tang = V(-math.sin(psi), math.cos(psi), 0.0)
        rings = []
        for r, chord, beta, kind in stations:
            be = math.radians(beta)
            cd = tang * math.cos(be) + V(0.0, 0.0, -math.sin(be))
            td = tang * math.sin(be) + V(0.0, 0.0, math.cos(be))
            if kind == "round":
                shape = [(xc, yc * 0.85) for xc, yc in circ]
            elif kind == "mix":
                shape = [(0.5 * (a[0] + c[0]), 0.5 * (a[1] * 2.2 + c[1] * 0.85)) for a, c in zip(loop, circ)]
            else:
                shape = loop
            origin = V(0.0, 0.0, PROP_Z) + radial * r - cd * (chord * 0.4)
            rings.append(section(origin, chord, cd, td, shape))
        mb.loft(rings, lambda c: "prop_tip" if math.hypot(c[0], c[1]) > PROP_R - 0.075 else "prop")
    return make_object("PROPELLER", mb, col, parent, pivot=pivot, smooth=45)


def build_prop_blur(col, parent, pivot):
    mb = MB()
    n = 48
    for j in range(n):
        a0, a1 = 2 * math.pi * j / n, 2 * math.pi * (j + 1) / n
        def p(r, a):
            return V(r * math.cos(a), r * math.sin(a), PROP_Z)
        mb.f([mb.v(p(0.12, a0)), mb.v(p(PROP_R, a0)), mb.v(p(PROP_R, a1)), mb.v(p(0.12, a1))], "blur")
    return make_object("PROPELLER_Blur", mb, col, parent, pivot=pivot, smooth=None,
                       normals=lambda p: V(0, 0, 1))


# ============================================================================
# cabin interior
# ============================================================================
SCREEN = dict(x0=-0.415, x1=-0.145, y0=0.000, y1=0.2025, z=0.489)   # 4:3, in front of the left seat
PANEL_Z = 0.50            # rear face of the instrument panel (towards the pilot)


def bulkhead(mb, z0, z1, mat="panel"):
    rings = []
    for z in (z0, z1):
        ring = fuselage_ring(z, n=48)
        c = centroid(ring)
        rings.append([c + (p - c) * 0.985 for p in ring])
    mb.loft(rings, mat)


def gauge(mb, cx, cy, r, arcs=(), ticks=12, needle_deg=40, mat_face="gauge_face"):
    """Round instrument: bezel, face, tick marks, coloured arcs, needle."""
    z = PANEL_Z
    c = V(cx, cy, z)
    n = 32

    def ring(rad, dz):
        return [c + V(rad * math.cos(2 * math.pi * j / n), rad * math.sin(2 * math.pi * j / n), dz) for j in range(n)]
    # bezel and recessed face as one closed solid: the back cap lies on the panel
    face_pick = lambda q: mat_face if (abs(q.z - (z - 0.003)) < 1e-4 and (q - c).xy.length < r * 0.5) else "black"
    mb.loft([ring(r + 0.008, 0), ring(r + 0.006, -0.007), ring(r, -0.007), ring(r, -0.003)], face_pick)
    zf = z - 0.0031
    toward = V(0, 0, -1)                     # markings stand up off the face towards the pilot

    def band(a0, a1, r0, r1, mat, thick=0.0003):
        pts = []
        for i in range(9):
            a = math.radians(a0 + (a1 - a0) * i / 8)
            pts.append(V(cx + (r0 + r1) / 2 * math.sin(a), cy + (r0 + r1) / 2 * math.cos(a), zf))
        hw = (r1 - r0) / 2
        mb.sweep(pts, mat=mat, up=toward, profile=[(0, -hw), (thick, -hw), (thick, hw), (0, hw)])

    def slab(p0, p1, hw, mat, thick=0.0004):
        mb.sweep([p0, p1], mat=mat, up=toward, profile=[(0, -hw), (thick, -hw), (thick, hw), (0, hw)])
    # arcs (start deg, end deg, material) measured clockwise from 12 o'clock
    for a0, a1, mat in arcs:
        band(a0, a1, r * 0.80, r * 0.90, mat)
    for i in range(ticks):
        a = math.radians(-135 + 270 * i / (ticks - 1))
        d = V(math.sin(a), math.cos(a), 0)
        slab(c + d * (r * 0.76) + V(0, 0, -0.0031), c + d * (r * 0.95) + V(0, 0, -0.0031), 0.0011, "gauge_mark",
             thick=0.0005)
    a = math.radians(needle_deg)
    d = V(math.sin(a), math.cos(a), 0)
    slab(c - d * (r * 0.15) + V(0, 0, -0.0042), c + d * (r * 0.85) + V(0, 0, -0.0042), 0.0015, "gauge_mark",
         thick=0.0006)
    mb.disc(c + V(0, 0, -0.005), (0, 0, 1), r * 0.10, 0.002, n=12, mat="black")


def toggle(mb, x, y, angle=20):
    mb.disc((x, y, PANEL_Z - 0.002), (0, 0, 1), 0.007, 0.004, n=6, mat="chrome")
    tip = V(x, y + 0.016 * math.sin(math.radians(angle)), PANEL_Z - 0.018 * math.cos(math.radians(angle)))
    mb.tube(V(x, y, PANEL_Z - 0.003), tip, 0.0025, 0.0018, n=6, mat="chrome")
    mb.ellipsoid(tip, 0.003, 0.003, 0.003, "chrome", nu=4, nv=6)


def knob(mb, x, y, r=0.011, depth=0.018, mat="black", z=PANEL_Z):
    mb.tube(V(x, y, z), V(x, y, z - depth * 0.4), r * 0.4, n=8, mat="metal")
    mb.tube(V(x, y, z - depth * 0.4), V(x, y, z - depth), r, r * 0.9, n=14, mat=mat)


def build_panel(mb):
    z = PANEL_Z
    # panel: shaped slab that follows the cabin cross-section
    rings = []
    ys = [-0.11, -0.08, -0.03, 0.05, 0.13, 0.21, 0.26, 0.285]
    for zz in (z, z + 0.04):
        left = [V(-min(inner_halfwidth(zz, y, 0.965), 0.53), y, zz) for y in ys]
        right = [V(min(inner_halfwidth(zz, y, 0.965), 0.53), y, zz) for y in reversed(ys)]
        rings.append(left + right)
    mb.loft(rings, "panel")
    # glareshield: a padded hood from the panel top forward to the windshield, its
    # width following the narrowing nose so it stays inside the glazing
    rings = []
    for zz in (0.484, 0.49, 0.52, 0.56, 0.60, 0.64, 0.68, 0.70):
        ytop = 0.300 + (zz - 0.484) * 0.21
        w = min(0.50, inner_halfwidth(zz, ytop + 0.014, 0.955))
        lip = 0.006 if zz < 0.50 else 0.0
        ring = []
        for i in range(13):                       # upper surface, left to right
            x = -w + 2 * w * i / 12
            sag = 0.010 * (1 - (x / w) ** 2)
            ring.append(V(x, ytop + 0.014 + sag, zz))
        for i in range(13):                       # lower surface, right to left
            x = w - 2 * w * i / 12
            sag = 0.010 * (1 - (x / w) ** 2)
            ring.append(V(x, ytop - lip + sag * 0.5, zz))
        rings.append(ring)
    mb.loft(rings, "black")
    # EFIS bezel with soft keys and knobs
    s = SCREEN
    mb.rbox(((s["x0"] + s["x1"]) / 2, (s["y0"] + s["y1"]) / 2 - 0.006, z - 0.0045),
            (s["x1"] - s["x0"] + 0.04, s["y1"] - s["y0"] + 0.052, 0.009), 0.006, "black")
    for i in range(7):
        x = s["x0"] + 0.015 + i * (s["x1"] - s["x0"] - 0.03) / 6
        mb.rbox((x, s["y0"] - 0.018, z - 0.012), (0.022, 0.009, 0.005), 0.002, "panel")
    for x in (s["x0"] - 0.002, s["x1"] + 0.002):
        knob(mb, x, s["y0"] - 0.018, r=0.008, depth=0.018, z=z - 0.011)
    # backup instruments: airspeed, altimeter, tachometer above the radio stack
    gy = 0.212
    gauge(mb, 0.035, gy, 0.031, arcs=((-120, -40, "arc_white"), (-80, 60, "arc_green"),
                                      (60, 110, "arc_yellow"), (110, 113, "arc_red")), needle_deg=-60)
    gauge(mb, 0.135, gy, 0.031, ticks=10, needle_deg=110)
    gauge(mb, 0.235, gy, 0.031, arcs=((-100, 70, "arc_green"), (70, 100, "arc_yellow"),
                                      (100, 104, "arc_red")), needle_deg=-120)
    gauge(mb, 0.335, gy, 0.031, arcs=((-60, 60, "arc_green"),), ticks=9, needle_deg=0)
    # COM radio and transponder
    for y0, y1, lcd in ((0.095, 0.165, (0.165, 0.285, 0.115, 0.148)), (0.020, 0.080, (0.165, 0.285, 0.035, 0.062))):
        mb.rbox((0.225, (y0 + y1) / 2, z - 0.006), (0.25, y1 - y0, 0.014), 0.004, "black")
        mb.rbox(((lcd[0] + lcd[1]) / 2, (lcd[2] + lcd[3]) / 2, z - 0.0135), (lcd[1] - lcd[0], lcd[3] - lcd[2], 0.002),
                0.0008, "lcd")
        knob(mb, 0.125, (y0 + y1) / 2, r=0.013, depth=0.03, z=z - 0.012)
        knob(mb, 0.125, (y0 + y1) / 2, r=0.008, depth=0.04, z=z - 0.012)
        knob(mb, 0.320, (y0 + y1) / 2, r=0.009, depth=0.024, z=z - 0.012)
        for i in range(4):
            mb.rbox((0.175 + i * 0.03, y0 + 0.008, z - 0.0135), (0.02, 0.007, 0.003), 0.001, "panel")
    # toggle switch row under the display, key switch and master, breakers, pull knobs
    for i in range(6):
        toggle(mb, -0.395 + i * 0.045, -0.055)
        mb.rbox((-0.395 + i * 0.045, -0.078, z - 0.0005), (0.03, 0.007, 0.001), 0.0, "gauge_mark")
    mb.disc((-0.475, 0.06, z - 0.003), (0, 0, 1), 0.016, 0.006, n=16, mat="chrome")
    mb.rbox((-0.475, 0.06, z - 0.012), (0.006, 0.026, 0.012), 0.002, "black")
    for i in range(6):
        x = 0.135 + i * 0.038
        mb.tube(V(x, -0.055, z), V(x, -0.055, z - 0.012), 0.0065, n=10, mat="black")
        mb.tube(V(x, -0.055, z - 0.012), V(x, -0.055, z - 0.016), 0.004, n=8, mat="gauge_mark")
    for x, mat in ((-0.08, "black"), (-0.035, "knob_red"), (0.36, "black")):
        knob(mb, x, -0.075, r=0.011, depth=0.03, mat=mat)
    mb.rbox((-0.06, -0.095, z - 0.0005), (0.07, 0.008, 0.001), 0.0, "placard")
    # throttle (push-pull) and its friction lock
    mb.tube(V(0.07, -0.045, z), V(0.07, -0.045, 0.43), 0.006, n=8, mat="chrome")
    mb.ellipsoid(V(0.07, -0.045, 0.42), 0.022, 0.022, 0.018, "black", nu=6, nv=14)
    mb.disc((0.07, -0.045, z - 0.003), (0, 0, 1), 0.012, 0.006, n=12, mat="metal")
    # compass on the glareshield
    mb.rbox((0.0, 0.345, 0.55), (0.07, 0.05, 0.06), 0.012, "black", rot=rot_x(-8))
    mb.rbox((0.0, 0.345, 0.519), (0.045, 0.025, 0.002), 0.004, "gauge_face")
    mb.rbox((0.0, 0.345, 0.5185), (0.003, 0.022, 0.001), 0.0, "arc_red")


def build_seat(mb, cx):
    tilt = rot_x(-14)       # back reclined: top goes aft
    base = V(cx, -0.43, -0.25)
    mb.rbox(base, (0.40, 0.07, 0.46), 0.025, "seat", rot=rot_x(5))
    mb.rbox(base + V(0, 0.05, 0.0), (0.34, 0.04, 0.42), 0.018, "seat_fabric", rot=rot_x(5))
    for dx in (-0.14, 0.14):                              # side bolsters
        mb.rbox(base + V(dx, 0.055, 0.0), (0.06, 0.05, 0.42), 0.022, "seat", rot=rot_x(5))
    pivot = V(cx, -0.40, -0.47)
    back_c = pivot + tilt @ V(0, 0.33, -0.05)
    mb.rbox(back_c, (0.40, 0.64, 0.07), 0.025, "seat", rot=tilt)
    mb.rbox(back_c + tilt @ V(0, -0.02, 0.04), (0.33, 0.54, 0.03), 0.014, "seat_fabric", rot=tilt)
    for dx in (-0.15, 0.15):
        mb.rbox(back_c + tilt @ V(dx, -0.04, 0.045), (0.05, 0.48, 0.05), 0.02, "seat", rot=tilt)
    # four-point harness lying on the seat, with buckle
    buckle = V(cx, -0.343, -0.20)
    front = tilt @ V(0, 0, 1)
    for dx in (-0.065, 0.065):
        # anchored behind the seat, over the top of the back, down the cushion to the buckle
        behind = pivot + tilt @ V(dx, 0.50, -0.092)
        rear_top = pivot + tilt @ V(dx, 0.655, -0.090)
        top = pivot + tilt @ V(dx, 0.668, -0.05)
        over = pivot + tilt @ V(dx, 0.655, 0.0095)
        down = pivot + tilt @ V(dx * 0.9, 0.12, 0.0095)
        crease = V(cx + dx * 0.8, -0.352, -0.42)
        pts = [behind, rear_top, top, over, down, crease, buckle + V(dx * 0.35, 0.0, -0.03)]
        back = tilt @ V(0, 0, -1)
        ups = [back, back, tilt @ V(0, 1, 0), front, front, V(0, 1, 0), V(0, 1, 0)]
        mb.sweep(pts, mat="belt", ups=ups, profile=[(0, -0.022), (0.003, -0.022), (0.003, 0.022), (0, 0.022)])
    for dx in (-0.2, 0.2):
        mb.sweep([V(cx + dx, -0.40, -0.40), V(cx + dx * 0.8, -0.35, -0.32), buckle + V(dx * 0.15, 0, 0)],
                 mat="belt", ups=[V(0, 1, 0)] * 3,
                 profile=[(0, -0.022), (0.003, -0.022), (0.003, 0.022), (0, 0.022)])
    mb.rbox(buckle + V(0, 0.004, 0), (0.06, 0.008, 0.06), 0.006, "chrome")


def build_controls(mb, cx):
    # stick with rubber boot, grip, PTT and trim hat
    base = V(cx, -0.485, 0.14)
    top = V(cx, -0.21, 0.185)
    mb.tube(base, top, 0.011, n=10, mat="metal")
    mb.tube(base, base + V(0, 0.07, 0.008), 0.05, 0.016, n=16, mat="rubber")
    grip0, grip1 = top, top + V(0, 0.115, 0.012)
    mb.tube(grip0, grip1, 0.017, 0.019, n=16, mat="black")
    for i in range(1, 4):
        p = grip0 + (grip1 - grip0) * (i / 4)
        mb.sweep([p + V(0.0195 * math.cos(a), 0, 0.0195 * math.sin(a)) for a in
                  [2 * math.pi * j / 16 for j in range(16)]], radius=0.0025, n=5, mat="black", closed=True)
    mb.ellipsoid(grip1 + V(0, 0.004, 0), 0.02, 0.012, 0.02, "black", nu=6, nv=14)
    mb.ellipsoid(grip1 + V(0, 0.012, 0.004), 0.005, 0.004, 0.005, "knob_red", nu=4, nv=8)
    mb.rbox(grip1 + V(0, -0.01, -0.016), (0.012, 0.012, 0.008), 0.002, "black")
    # rudder pedals with toe-brake plates
    for dx in (-0.085, 0.085):
        x = cx + dx
        piv = V(x, -0.475, 0.70)
        plate = V(x, -0.38, 0.725)
        mb.tube(piv, plate, 0.008, n=8, mat="metal")
        mb.rbox(plate + V(0, 0.01, -0.004), (0.065, 0.11, 0.014), 0.006, "black", rot=rot_x(-20))
        for k in range(4):
            mb.rbox(plate + V(0, -0.035 + k * 0.024, -0.013), (0.06, 0.004, 0.004), 0.0012, "metal",
                    rot=rot_x(-20))
    mb.tube(V(cx - 0.12, -0.475, 0.70), V(cx + 0.12, -0.475, 0.70), 0.009, n=8, mat="metal")


def build_frame(mb):
    """Visible steel tube cage: roof bows, door frames, sills, A-pillars."""
    s = 0.962
    mb.sweep([fus_point(0.285, a, s) for a in range(18, 163, 6)], radius=0.012, n=10, mat="frame")
    mb.sweep([fus_point(-0.62, a, s) for a in range(54, 127, 6)], radius=0.012, n=10, mat="frame")
    for side in (-1, 1):
        def sp(z, a):
            return fus_point(z, a if side > 0 else 180 - a, s)
        mb.sweep([sp(z, 47) for z in [0.80 - i * 0.05 for i in range(11)]], radius=0.012, n=10, mat="frame")
        mb.sweep([sp(z, 52.5) for z in [0.29 - i * 0.07 for i in range(17)]], radius=0.012, n=10, mat="frame")
        mb.sweep([sp(z, -25) for z in [0.78 - i * 0.07 for i in range(24)]], radius=0.014, n=10, mat="frame")
        mb.sweep([sp(-0.86, a) for a in range(-40, 58, 7)], radius=0.012, n=10, mat="frame")
        # door: inside handle and the edge of the glazing frame
        h = sp(-0.62, -10)
        mb.rbox(h + V(-side * 0.012, 0, 0), (0.018, 0.022, 0.12), 0.006, "black")


def build_cabin_interior(col, parent):
    mb = MB()
    bulkhead(mb, 0.90, 0.88, "panel")                   # firewall
    bulkhead(mb, -0.94, -0.96, "liner")                 # aft bulkhead
    # carpeted floor that follows the cabin width, rising into the footwell
    zs = [-0.94, -0.6, -0.2, 0.2, 0.5, 0.65, 0.80, 0.86]
    top, bot = [], []
    for z in zs:
        y = -0.485 if z < 0.62 else -0.485 + (z - 0.62) * 0.12
        w = inner_halfwidth(z, y, 0.975)
        top.append((V(-w, y, z), V(w, y, z)))
    rings = []
    for (l, r) in top:
        rings.append([l, r, r - V(0, 0.015, 0), l - V(0, 0.015, 0)])
    mb.loft(rings, "carpet")
    # centre console with fuel valve and park brake
    mb.rbox((0, -0.462, -0.05), (0.10, 0.05, 0.72), 0.012, "panel")
    mb.disc((0, -0.436, 0.16), (0, 1, 0), 0.02, 0.008, n=16, mat="black")
    mb.rbox((0, -0.428, 0.16), (0.008, 0.012, 0.05), 0.003, "knob_red")
    mb.rbox((0, -0.436, 0.16), (0.075, 0.001, 0.075), 0.0, "placard", rot=rot_y(45))
    mb.tube(V(0.0, -0.437, 0.25), V(0.0, -0.37, 0.30), 0.006, n=8, mat="metal")
    mb.ellipsoid((0.0, -0.365, 0.303), 0.012, 0.012, 0.02, "black", nu=4, nv=8)
    # flap lever with notched quadrant, and the trim lever beside it
    mb.rbox((0, -0.43, -0.22), (0.012, 0.07, 0.16), 0.004, "metal")
    for i in range(4):
        a = math.radians(30 + i * 15)
        mb.rbox((0, -0.43 + 0.06 * math.sin(a), -0.27 + 0.06 * math.cos(a)), (0.014, 0.006, 0.006), 0.001, "black")
    mb.tube(V(0.0, -0.44, -0.26), V(0.0, -0.30, -0.16), 0.008, n=8, mat="metal")
    mb.tube(V(0.0, -0.30, -0.16), V(0.0, -0.26, -0.13), 0.016, 0.014, n=12, mat="knob_red")
    mb.tube(V(0.035, -0.44, -0.30), V(0.035, -0.36, -0.25), 0.005, n=8, mat="metal")
    mb.ellipsoid((0.035, -0.355, -0.247), 0.01, 0.012, 0.018, "black", nu=4, nv=8)
    # baggage shelf and a fire extinguisher behind the seats
    w = inner_halfwidth(-0.78, -0.30, 0.95)
    mb.rbox((0, -0.31, -0.78), (2 * w, 0.02, 0.30), 0.006, "carpet")
    mb.rbox((0, -0.27, -0.635), (2 * w * 0.98, 0.07, 0.012), 0.004, "panel")
    mb.tube(V(0.30, -0.30, -0.80), V(0.30, -0.10, -0.80), 0.035, n=16, mat="knob_red")
    mb.tube(V(0.30, -0.10, -0.80), V(0.30, -0.075, -0.80), 0.014, n=10, mat="black")
    mb.tube(V(0.30, -0.08, -0.80), V(0.30, -0.08, -0.74), 0.005, n=6, mat="black")
    for y in (-0.26, -0.16):
        mb.sweep([V(0.30 + 0.039 * math.cos(a), y, -0.80 + 0.039 * math.sin(a)) for a in
                  [2 * math.pi * j / 16 for j in range(16)]], radius=0.004, n=5, mat="metal", closed=True)
    # fuel sight gauges at the wing roots, and the overhead skylight sunshade frame
    for side in (-1, 1):
        x = side * 0.36
        mb.tube(V(x, 0.42, 0.05), V(x, 0.58, 0.05), 0.008, n=10, mat="glass")
        mb.tube(V(x, 0.44, 0.05), V(x, 0.53, 0.05), 0.0065, n=8, mat="placard")
        for y in (0.42, 0.58):
            mb.rbox((x, y, 0.05), (0.03, 0.014, 0.03), 0.004, "metal")
    build_panel(mb)
    build_frame(mb)
    for side in (-1, 1):
        cx = side * 0.26
        build_seat(mb, cx)
        build_controls(mb, cx)
    return make_object("Cabin", mb, col, parent, smooth=40)


def build_screen(col, parent):
    mb = MB()
    s = SCREEN
    mb.f([mb.v((s["x0"], s["y0"], s["z"])), mb.v((s["x1"], s["y0"], s["z"])),
          mb.v((s["x1"], s["y1"], s["z"])), mb.v((s["x0"], s["y1"], s["z"]))],
         SCREEN_MATERIAL, uv=[(0, 1), (1, 1), (1, 0), (0, 0)])
    return make_object("EFIS_Screen", mb, col, parent, smooth=None, normals=lambda p: V(0, 0, -1))


# ============================================================================
# scene
# ============================================================================
def animate(obj, axis_index, keys, name):
    """Rotation keys (frame, degrees) about a Blender axis, linear, as action `name`."""
    obj.rotation_mode = "XYZ"
    for frame, deg in keys:
        rot = [0.0, 0.0, 0.0]
        rot[axis_index] = math.radians(deg)
        obj.rotation_euler = rot
        obj.keyframe_insert("rotation_euler", index=axis_index, frame=frame)
    act = obj.animation_data.action
    act.name = name
    for fc in _fcurves(act):
        for kp in fc.keyframe_points:
            kp.interpolation = "LINEAR"
    obj.rotation_euler = (0.0, 0.0, 0.0)


def _fcurves(action):
    if hasattr(action, "fcurves") and len(getattr(action, "fcurves", [])):
        return action.fcurves
    out = []
    for layer in getattr(action, "layers", []):
        for strip in layer.strips:
            for bag in strip.channelbags:
                out.extend(bag.fcurves)
    return out


def setup_export_presets(scene, ext_col, int_col):
    from io_scene_gltf2_msfs_2024.io.exp import export_settings as es
    es.init_setting_presets(scene)
    st = es.get_active_export_settings(scene)
    st.export_apply = True                    # apply any modifiers you add later
    st.export_animations = True
    rel = os.path.relpath(MODEL_DIR, os.path.dirname(BLEND_PATH))
    for name, col in (("A240", ext_col), ("A240_interior", int_col)):
        p = scene.msfs_multi_exporter_presets.add()
        p.name = name
        p.preset_name = name
        p.folder_path = "//" + rel.replace("\\", "/") + "/"
        p.enabled = True
        layer = p.layers.add()
        layer.collection = col
        layer.enabled = True
        layer.preset_id = p.name
    scene.msfs_export_mode = "PRESETS"


def main():
    enable_msfs_exporter()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    enable_msfs_exporter()
    scene = bpy.context.scene
    scene.name = "A240"
    scene.render.fps = FPS
    scene.frame_start, scene.frame_end = 0, 100
    scene.frame_set(0)
    scene.unit_settings.system = "METRIC"

    ext = bpy.data.collections.new("Exterior")
    inn = bpy.data.collections.new("Interior")
    scene.collection.children.link(ext)
    scene.collection.children.link(inn)

    root = make_empty("A240_Exterior", ext, size=0.5)
    fus = build_fuselage(ext, root)
    build_fuselage_details(ext, root, fus)
    build_stabilizer(ext, root)
    build_fin(ext, root)
    build_spinner(ext, root)
    for side in (-1, 1):
        build_wing(side, ext, root)
        build_struts(side, ext, root)
    build_gear(ext, root)

    # control surfaces: outer node carries the flap rotation, inner node the aileron rotation
    flaps, ails = {}, {}
    for side, tag in ((-1, "L"), (1, "R")):
        flap = make_empty("SKEL_Flap_%s" % tag, ext, root, FLAPERON_PIVOT)
        ails[tag] = build_flaperon(side, ext, flap, FLAPERON_PIVOT)
        flaps[tag] = flap
    elevator = build_elevator(ext, root, (0.0, STAB_Y, ELEV_HINGE_Z))
    rudder = build_rudder(ext, root, (0.0, 0.5, RUDDER_HINGE_Z))
    # The sim spins these two itself (engines.cfg prop_node.0) and the behaviour swaps
    # the solid blades for the blur disc once the propeller turns.
    build_blades(ext, root, (0.0, 0.0, PROP_Z))
    build_prop_blur(ext, root, (0.0, 0.0, PROP_Z))

    # Animations, 0..100 (50 = neutral for the controls, 0 = retracted for the flaps).
    # Blender X = glTF X (left wing): +angle lifts a trailing edge. Blender Z = glTF Y (up).
    animate(elevator, 0, [(0, -20), (50, 0), (100, 28)], "elevator_percent_key")
    animate(rudder, 2, [(0, -25), (50, 0), (100, 25)], "rudder_percent_key")
    animate(ails["L"], 0, [(0, 20), (50, 0), (100, -15)], "l_aileron_percent_key")
    animate(ails["R"], 0, [(0, -15), (50, 0), (100, 20)], "r_aileron_percent_key")
    animate(flaps["L"], 0, [(0, 0), (100, -20)], "l_flap_percent_key")
    animate(flaps["R"], 0, [(0, 0), (100, -20)], "r_flap_percent_key")

    iroot = make_empty("A240_Interior", inn, size=0.3)
    liner = build_liner(inn, iroot)
    build_cabin_interior(inn, iroot)
    build_screen(inn, iroot)

    setup_export_presets(scene, ext, inn)
    scene.frame_set(0)
    os.makedirs(os.path.dirname(BLEND_PATH), exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=BLEND_PATH, compress=True)
    tris = 0
    for o in bpy.data.objects:
        if o.type == "MESH":
            tris += sum(len(p.vertices) - 2 for p in o.data.polygons)
    print("saved %s: %d objects, ~%d triangles" % (BLEND_PATH, len(bpy.data.objects), tris))


if __name__ == "__main__":
    main()
