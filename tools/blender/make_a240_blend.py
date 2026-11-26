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


def skin_normal(z, phi_deg):
    """Outward unit normal of the (smooth) skin at (z, phi), including its slope along z."""
    around = fus_point(z, phi_deg + 0.5) - fus_point(z, phi_deg - 0.5)
    along = fus_point(z + 0.005, phi_deg) - fus_point(z - 0.005, phi_deg)
    n = around.cross(along).normalized()
    return n if n.dot(fus_out(fus_point(z, phi_deg))) > 0 else -n


def skin_frame(z, phi_deg):
    """3x3 frame on the skin at (z, phi): columns = outward, around (phi increasing), along +z."""
    p = fus_point(z, phi_deg)
    along = (fus_point(z + 0.01, phi_deg) - fus_point(z - 0.01, phi_deg)).normalized()
    out = fus_out(p)
    out = (out - along * out.dot(along)).normalized()
    return Matrix((out, along.cross(out), along)).transposed()


def fuselage_ring(z, n=RING_N, scale=1.0):
    return [fus_point(z, 360.0 * j / n, scale) for j in range(n)]


def ring_halfwidth(z, y, scale=1.0):
    """Half width at height y of the polygon fuselage_ring(z, scale) - the shape the mesh
    really has. Near the belly and the roof, where the section is shallow, the polygon's
    flat sides lie several mm inside the smooth curve (fus_point)."""
    ring = fuselage_ring(z, scale=scale)
    best = 0.0
    for a, b in zip(ring, ring[1:] + ring[:1]):
        if a.x >= 0 and b.x >= 0 and a.y != b.y and (a.y - y) * (b.y - y) <= 0:
            best = max(best, a.x + (b.x - a.x) * (y - a.y) / (b.y - a.y))
    return best


def wall_x(z, y):
    """Where an interior part should end to meet the cabin wall at height y: 1.5 mm past
    the lining (so no crack shows) and still well inside the skin (so nothing pokes out)."""
    return ring_halfwidth(z, y, LINER_SCALE) + 0.0015


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


# The lining sits this far inside the skin (as a fraction of the section), and runs from
# inside the aft bulkhead to inside the firewall so its ends never show a crack.
LINER_SCALE = 0.992
LINER_Z = (-0.95, 0.89)
BULKHEAD_SCALE = 0.996    # bulkheads reach past the lining, so the lining runs into them
REVEAL_SCALE = 1.003      # window reveals end just outside the skin, inside the rubber seal


def build_liner(col, parent):
    """Cabin lining: the skin between the bulkheads, just inside it, facing inwards.

    Glazed panels are left open so the pilot sees out. Every opening gets a reveal: a strip
    from the lining's edge out through the skin, facing into the opening. Without it the
    few millimetres between the lining and the glass show daylight when you lean towards
    a window in VR (the sim draws neither side's back faces)."""
    z0, z1 = LINER_Z
    zs = [z1] + [z for z in fuselage_stations() if z0 + 1e-6 < z < z1 - 1e-6] + [z0]
    n = RING_N
    mb = MB()
    inner = [[mb.v(p) for p in fuselage_ring(z, scale=LINER_SCALE)] for z in zs]
    outer = {}

    def outer_v(r, c):                    # skin-side corner of a reveal, made when needed
        if (r, c) not in outer:
            outer[(r, c)] = mb.v(fus_point(zs[r], 360.0 * c / n, REVEAL_SCALE))
        return outer[(r, c)]

    def quad(i, j):
        k = (j + 1) % n
        return [inner[i][j], inner[i][k], inner[i + 1][k], inner[i + 1][j]]

    glazed = {}
    for i in range(len(zs) - 1):
        for j in range(n):
            c = centroid([mb.verts[q] for q in quad(i, j)])
            glazed[(i, j)] = fuselage_material(c).startswith("glass")
    for (i, j), is_glass in glazed.items():
        if is_glass:
            continue
        k = (j + 1) % n
        q = quad(i, j)
        c = centroid([mb.verts[x] for x in q])
        mb.f_toward(q, "liner", fus_center(c[2]) - c)
        # the four edges of this panel: (neighbour across it, edge as (row, col) pairs)
        for nb, edge in (((i - 1, j), ((i, j), (i, k))), ((i + 1, j), ((i + 1, j), (i + 1, k))),
                         ((i, (j - 1) % n), ((i, j), (i + 1, j))), ((i, k), ((i, k), (i + 1, k)))):
            if not glazed.get(nb):
                continue
            (ra, ca), (rb, cb) = edge
            strip = [inner[ra][ca], inner[rb][cb], outer_v(rb, cb), outer_v(ra, ca)]
            mid = centroid([mb.verts[x] for x in strip])
            mb.f_toward(strip, "liner", centroid([mb.verts[x] for x in quad(*nb)]) - mid)
    return make_object("Cabin_Liner", mb, col, parent, smooth=30, normals=None)


def panel_line(mb, pts, width=0.004, height=0.0012, mat="black", closed=False):
    """A thin dark strip standing `height` proud of the skin along pts (skin points). Its
    foot is sunk 1 mm into the skin, so no gap shows under its edges."""
    outs = [fus_out(p) for p in pts]
    sunk = [V(p) - o * 0.001 for p, o in zip(pts, outs)]
    h = height + 0.001
    prof = [(0, -width / 2), (h, -width / 2), (h, width / 2), (0, width / 2)]
    mb.sweep(sunk, mat=mat, profile=prof, ups=outs, closed=closed)


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
                   [dp(z, 52) for z in [0.29 - i * 0.06 for i in range(19)]] +
                   [dp(-0.84, a) for a in range(52, -28, -4)] +
                   [dp(z, -27) for z in [-0.84 + i * 0.06 for i in range(19)]])
        panel_line(mb, outline, closed=True)
        for z in (0.12, -0.62):
            p = dp(z, 53.5)
            o = fus_out(p)
            # sunk 2.5 mm into the skin: any deeper and it shows through the cabin lining
            mb.tube(p + o * 0.0065 + V(0, 0, 0.035), p + o * 0.0065 - V(0, 0, 0.035), 0.009, n=10, mat="metal")
        hp = dp(-0.70, -14)
        o = fus_out(hp)
        mb.sweep([hp + o * 0.002, hp + o * 0.022, hp + o * 0.022 - V(0, 0, 0.10)], radius=0.008,
                 n=8, mat="black")
    # fresh-air scoops either side of the windshield base, laid along the tapering skin
    # and sunk only 3 mm into it (deeper and they would show through the cabin lining)
    for side in (-1, 1):
        a = 8 if side > 0 else 172
        p = fus_point(0.86, a)
        mb.ellipsoid(p + skin_frame(0.86, a).col[0] * 0.009, 0.012, 0.022, 0.05, "black", nu=6, nv=10,
                     rot=skin_frame(0.86, a))
    # exhaust: tailpipe out of the cowl exit, slightly to the right
    mb.sweep([(0.08, -0.40, 1.06), (0.09, -0.50, 1.01), (0.10, -0.555, 0.94)], radius=0.021, n=14,
             mat="metal_dark")
    mb.sweep([(-0.08, -0.40, 1.07), (-0.08, -0.52, 1.03)], radius=0.016, n=12, mat="metal_dark")
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


def lining_y(x, z):
    """Height of the cabin lining's ceiling at lateral position x, station z."""
    cy = fus_center(z).y
    return cy + LINER_SCALE * (roof_y(x / LINER_SCALE, z) - cy)


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
    # root fairing: the root section carried inboard over the cabin roof. Its underside is
    # sunk 2 mm into the roof - no crack between wing and cabin, yet clear of the cabin
    # lining - and its top stays above that, so every section is a clean outline (the old
    # one folded over itself and showed through the skylight). The outer station comes
    # down onto the strip of roof above the door window, closing the slot under the wing
    # root. The inner and outer stations follow the cabin's taper so the fairing never
    # overhangs the skylight or the door glass (seen through them, a sunk edge would show
    # the inside of the wing).
    root = rings[0]
    n_up = (len(root) + 2) // 2            # upper surface TE -> LE, then lower LE -> TE
    fairing = []
    for station in ("skylight", 0.44, "door"):
        ring = []
        for k, rp in enumerate(root):
            if station == "skylight":
                x = max(0.38, fus_point(rp.z, 55.5).x)
            elif station == "door":
                x = min(0.49, fus_point(rp.z, 46.5).x)
            else:
                x = station
            base = roof_y(x, rp.z)
            ring.append(V(side * x, base - 0.002 if k >= n_up else max(rp.y, base + 0.006), rp.z))
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
    # fold fittings at the rear spar root (just outboard of the fairing), strut fittings
    for xc in (0.62,):
        pt, nrm = wing_point(side, 0.53, xc, False)
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
        mb.rbox(le + V(0, -0.001, 0.006), (0.02, 0.003, 0.025), 0.001, "metal")      # tab out of the LE
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
        mb.sweep([c + V(0.0215 * math.cos(a), 0, 0.0215 * math.sin(a))
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
# The instrument panel is Aeropro's current Garmin panel for the Rotax 915 iS, traced
# from the factory's head-on render (aeropro.sk/eurofox, image "panel-garmin-1a", 2025).
# The render was scaled from two parts of known size, the G3X Touch GDU 460 bezel
# (10.85 x 7.82 in) and the 6.25 in wide Garmin COM, which agree at 1.83 px per mm.
# Positions on the panel are (x, d) in mm: x to the right of the centreline as the pilot
# sees it, d down from the top centre of the panel.
PANEL_Z = 0.50            # rear face of the instrument panel (towards the pilot)
PANEL_TOP_Y = 0.290       # top centre of the panel
PANEL_T = 0.006           # panel sheet thickness
# outline, right half, from the top centre round to the bottom corner: (half width, d)
PANEL_OUTLINE = [(0, 0), (61, 0.5), (105, 2.2), (149, 4.4), (192, 8.7), (236, 14.8), (262, 19.7),
                 (287, 26.2), (307, 32.8), (324, 39.3), (339, 45.9), (351, 52.5), (362, 59.0),
                 (372, 65.6), (381, 72.1), (390, 78.7), (398, 85.2), (405, 91.8), (411.5, 98.4),
                 (417, 104.9), (422, 111.5), (427, 118.0), (431, 124.6), (435, 131.1), (438, 137.7),
                 (441, 144.3), (443, 150.8), (445, 157.4), (446.5, 164.0), (447.5, 172.0), (448, 185),
                 (449, 205), (450, 225), (452, 245), (453.5, 262), (454, 272), (451, 278.5),
                 (446, 280.3)]
RIM_END_D = 172.0         # the black edge roll along the top ends this far down the sides
# G3X Touch GDU 460: bezel 275.6 x 198.6 mm; the 10.6 in 5:3 screen (1280 x 768) is
# 230.9 x 138.5 mm, centred across the bezel and 7.4 mm above its centre.
G3X = dict(x=-229.5, d=166.4, w=275.6, h=198.6)
G3X_SCREEN = dict(x=-229.5, d=159.0, w=230.9, h=138.5)
_S = G3X_SCREEN
SCREEN = dict(x0=(_S["x"] - _S["w"] / 2) / 1000, x1=(_S["x"] + _S["w"] / 2) / 1000,
              y0=PANEL_TOP_Y - (_S["d"] + _S["h"] / 2) / 1000, y1=PANEL_TOP_Y - (_S["d"] - _S["h"] / 2) / 1000,
              z=PANEL_Z - 0.0065)
G3X_FACE_Z = PANEL_Z - 0.010     # front of the bezel frame


def pp(x, d, dz=0.0):
    """Panel position (mm right, mm down from the top centre) -> sim point, dz along +z."""
    return V(x / 1000.0, PANEL_TOP_Y - d / 1000.0, PANEL_Z + dz)


def bulkhead(mb, z0, z1, mat="panel"):
    """A bulkhead plate from z0 to z1. It is cut from the same ring as the cabin lining but
    a little larger, so the lining runs into it and no daylight shows round its edge."""
    mb.loft([fuselage_ring(z, scale=BULKHEAD_SCALE) for z in (z0, z1)], mat)


def rounded_poly(pts, r, n=3):
    """Round the corners of a convex 2D polygon (mm) with radius-ish r (quadratic blends)."""
    out = []
    m = len(pts)
    for i in range(m):
        p, a, b = V(*pts[i]), V(*pts[i - 1]), V(*pts[(i + 1) % m])
        ra = min(r, (a - p).length * 0.45)
        rb = min(r, (b - p).length * 0.45)
        p0 = p + (a - p).normalized() * ra
        p1 = p + (b - p).normalized() * rb
        for k in range(n + 1):
            t = k / n
            q = p0 * (1 - t) ** 2 + p * 2 * t * (1 - t) + p1 * t * t
            out.append((q.x, q.y))
    return out


def flat_poly(mb, pts, z0, z1, mat):
    """Slab between z0 and z1 whose outline is a convex polygon of panel (x, d) points in mm."""
    rings = [[pp(x, d, z - PANEL_Z) for x, d in pts] for z in (z0, z1)]
    mb.loft(rings, mat)


def plate(mb, x, d, w, h, mat="label", lines=0, dz=0.0):
    """Label or placard: a thin plate standing proud of the panel, with faint 'text' lines."""
    depth = 0.0008
    mb.rbox(pp(x, d, dz - depth / 2), (w / 1000.0, h / 1000.0, depth), 0.0004, mat)
    for i in range(lines):
        ly = d - h / 2 + h * (i + 0.8) / (lines + 0.6)
        lw = w * (0.7 if lines == 1 else (0.82 if i % 3 else 0.6))
        lx = x if lines == 1 else x - (w - lw) / 2 + w * 0.05          # one word centred, text left-aligned
        mb.rbox(pp(lx, ly, dz - depth - 0.0002), (lw / 1000.0 * 0.9, min(0.0011, h / 1000.0 / (lines * 2.2)), 0.0003),
                0.0, "label_text")


def led(mb, x, d, lens="lens_red", r=0.0028, dz=0.0):
    c = pp(x, d, dz)
    mb.disc(c + V(0, 0, -0.0015), (0, 0, 1), r + 0.0018, 0.003, n=12, mat="chrome")
    mb.ellipsoid(c + V(0, 0, -0.003), r, r, r * 0.8, lens, nu=4, nv=10)


def screw(mb, x, d, dz=0.0):
    mb.disc(pp(x, d, dz - 0.0008), (0, 0, 1), 0.0028, 0.0016, n=10, mat="metal_dark")


def gauge(mb, cx, cy, r, arcs=(), ticks=12, needle_deg=40, mat_face="gauge_face", span=(-135, 135),
          bezel=0.008):
    """Round instrument: bezel, face, tick marks, coloured arcs, needle. Angles are degrees
    clockwise from 12 o'clock."""
    z = PANEL_Z
    c = V(cx, cy, z)
    n = 32

    def ring(rad, dz):
        return [c + V(rad * math.cos(2 * math.pi * j / n), rad * math.sin(2 * math.pi * j / n), dz) for j in range(n)]
    # bezel and recessed face as one closed solid: the back cap lies on the panel
    face_pick = lambda q: mat_face if (abs(q.z - (z - 0.003)) < 1e-4 and (q - c).xy.length < r * 0.5) else "black"
    mb.loft([ring(r + bezel, 0), ring(r + bezel - 0.002, -0.007), ring(r, -0.007), ring(r, -0.003)], face_pick)
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
    for a0, a1, mat in arcs:
        band(a0, a1, r * 0.80, r * 0.90, mat)
    for i in range(ticks):
        a = math.radians(span[0] + (span[1] - span[0]) * i / max(ticks - 1, 1))
        d = V(math.sin(a), math.cos(a), 0)
        slab(c + d * (r * 0.76) + V(0, 0, -0.0031), c + d * (r * 0.95) + V(0, 0, -0.0031), 0.0011, "gauge_mark",
             thick=0.0005)
    a = math.radians(needle_deg)
    d = V(math.sin(a), math.cos(a), 0)
    slab(c - d * (r * 0.15) + V(0, 0, -0.0042), c + d * (r * 0.85) + V(0, 0, -0.0042), 0.0015, "gauge_mark",
         thick=0.0006)
    mb.disc(c + V(0, 0, -0.0045), (0, 0, 1), r * 0.10, 0.0032, n=12, mat="black")    # hub, on the dial


# The panel's switches, buttons and key: build_panel() puts their fixed parts (bushings,
# bezels) in the cabin mesh and lists their moving parts here; build_panel_controls() turns
# each into its own node, keyed 0..100. Node and animation share a name.
PANEL_PARTS = []
TOGGLE_THROW = 40.0       # lever travel from OFF (20 deg down) to ON (20 deg up), deg


def panel_part(name, mb, pivot, kind, axis, keys):
    """kind "rot": keys are (key, degrees) about Blender axis `axis`; "slide": (key, metres)."""
    PANEL_PARTS.append(dict(name=name, mb=mb, pivot=V(pivot), kind=kind, axis=axis, keys=keys))


def toggle(mb, x, y, angle=20, z=PANEL_Z, name=None, length=0.018, ball=0.003):
    """Toggle switch: hex bushing on the panel and a lever. With a name, the lever is a
    moving part that pivots in the bushing: key 0 = OFF (down), 100 = ON (up)."""
    mb.disc((x, y, z - 0.002), (0, 0, 1), 0.007, 0.004, n=6, mat="chrome")
    lever = MB() if name else mb
    a = math.radians(angle)
    tip = V(x, y + length * 0.89 * math.sin(a), z - length * math.cos(a))
    lever.tube(V(x, y, z - 0.003), tip, 0.0025, 0.0018, n=6, mat="chrome")
    lever.ellipsoid(tip, ball, ball, ball, "chrome", nu=4, nv=6)
    if name:
        # +X about Blender X lifts a lever that points at the pilot
        panel_part(name, lever, (x, y, z - 0.003), "rot", 0, [(0, 0), (100, TOGGLE_THROW)])


def knob(mb, x, y, r=0.011, depth=0.018, mat="black", z=PANEL_Z):
    mb.tube(V(x, y, z), V(x, y, z - depth * 0.4), r * 0.4, n=8, mat="metal")
    mb.tube(V(x, y, z - depth * 0.4), V(x, y, z - depth), r, r * 0.9, n=14, mat=mat)


def panel_switch(mb, x, d, name, label_w=20.0, label_dd=-9.5, off_dd=10.5):
    """Toggle (modelled OFF, lever down) with its name plate above and OFF plate below."""
    c = pp(x, d)
    toggle(mb, c.x, c.y, angle=-20, name=name)
    plate(mb, x, d + label_dd, label_w, 5.0, lines=1)
    plate(mb, x, d + off_dd, 11.0, 4.5, lines=1)


def asi_angle(kt):
    """The airspeed dial in the render: 40 kt at 1 o'clock, 240 kt at 11 o'clock."""
    return kt / 40.0 * 25.0 if kt < 40 else 25.0 + (kt - 40.0) * 1.525


def build_g3x(mb):
    """Garmin G3X Touch GDU 460: bezel frame round a recessed screen, knobs and keys below."""
    g, s = G3X, G3X_SCREEN
    bx0, bx1 = g["x"] - g["w"] / 2, g["x"] + g["w"] / 2
    bd0, bd1 = g["d"] - g["h"] / 2, g["d"] + g["h"] / 2
    sx0, sx1 = s["x"] - s["w"] / 2, s["x"] + s["w"] / 2
    sd0, sd1 = s["d"] - s["h"] / 2, s["d"] + s["h"] / 2
    mb.rbox(pp(g["x"], g["d"], -0.003), (g["w"] / 1000, g["h"] / 1000, 0.006), 0.004, "black")
    fz, fd = -0.008, 0.004            # frame centre and depth (front face at G3X_FACE_Z)
    for (x0, x1, d0, d1) in ((bx0, bx1, bd0, sd0), (bx0, bx1, sd1, bd1), (bx0, sx0, sd0, sd1), (sx1, bx1, sd0, sd1)):
        mb.rbox(pp((x0 + x1) / 2, (d0 + d1) / 2, fz), ((x1 - x0) / 1000, (d1 - d0) / 1000, fd), 0.0012, "bezel")
    kd = bd1 - 16.5                   # control row
    zf = G3X_FACE_Z
    for dx in (-115.0, 115.0):
        c = pp(g["x"] + dx, kd)
        knob(mb, c.x, c.y, r=0.0105, depth=0.009, z=zf)
        knob(mb, c.x, c.y, r=0.0068, depth=0.017, z=zf)
    for dx in (-85.0, -58.0, 56.0, 83.0):
        mb.rbox(pp(g["x"] + dx, kd, fz - 0.0035), (0.016, 0.008, 0.003), 0.0015, "button")
    for dx in (-70.5, 70.5):
        mb.disc(pp(g["x"] + dx, kd, fz - 0.002), (0, 0, 1), 0.0016, 0.001, n=8, mat="lcd")
    mb.rbox(pp(g["x"], kd, fz - 0.0021), (0.025, 0.0025, 0.0004), 0.0, "inlet_dark")      # SD card slot


GLARESHIELD_Z = (0.484, 0.49, 0.52, 0.56, 0.60, 0.64, 0.68, 0.71, 0.74, 0.77, 0.80, 0.83, 0.86, 0.885)


def glareshield_section(zz):
    """Cross-section of the glareshield at station zz: upper surface left to right, then
    the lower surface back. Its edges stay below the windshield's lower edge and its top
    below the glass, so it closes the cabin off from the space behind the panel."""
    ytop = 0.300 + (zz - 0.484) * 0.21
    lip = 0.006 if zz < 0.50 else 0.0
    up_e = min(ytop + 0.014, fus_point(zz, 45).y - 0.010)   # edge height on the wall
    lo_e = min(ytop - lip, up_e - 0.012)
    w_up = wall_x(zz, up_e)
    w_lo = wall_x(zz, lo_e)

    def upper(x):
        return min(up_e + 0.010 * (1 - (x / w_up) ** 2), roof_y(x, zz) - 0.005)

    ring = []
    for i in range(13):                           # upper surface, left to right
        x = -w_up + 2 * w_up * i / 12
        ring.append(V(x, upper(x), zz))
    for i in range(13):                           # lower surface, right to left
        x = w_lo - 2 * w_lo * i / 12
        ring.append(V(x, min(lo_e + 0.005 * (1 - (x / w_lo) ** 2), upper(min(abs(x), w_up)) - 0.012), zz))
    return ring


ROW_SWITCHES = ("Light_Landing", "Light_Nav", "Light_Strobe", "COM", "TXP", "FLARM", "GDU460", "Spare", "USB")


def build_panel_controls(col, parent):
    """The panel's switches, buttons, guard and key as nodes, each keyed 0..100."""
    for part in PANEL_PARTS:
        obj = make_object(part["name"], part["mb"], col, parent, pivot=part["pivot"], smooth=40)
        if part["kind"] == "rot":
            animate(obj, part["axis"], part["keys"], part["name"])
        else:
            animate_slide(obj, part["axis"], part["keys"], part["name"])


def build_panel(mb):
    z = PANEL_Z
    PANEL_PARTS.clear()
    # black backing that closes the cabin cross-section behind and beside the panel; its
    # sides run just past the cabin lining, so there is no crack at the walls
    rings = []
    ys = [0.004, 0.03, 0.08, 0.13, 0.18, 0.23, 0.27, 0.302]
    for zz in (z + PANEL_T, z + 0.04):
        left = [V(-wall_x(zz, y), y, zz) for y in ys]
        right = [V(wall_x(zz, y), y, zz) for y in reversed(ys)]
        rings.append(left + right)
    mb.loft(rings, "black")
    # the panel itself: grey sheet with the arched top of the render
    right = list(PANEL_OUTLINE)
    outline = [(-x, d) for x, d in reversed(right[1:])] + right
    flat_poly(mb, outline, z, z + PANEL_T, "panel_grey")
    # black edge roll along the top, deepening towards the sides
    arc = [(x, d) for x, d in outline if d <= RIM_END_D]
    rings = []
    for i, (x, d) in enumerate(arc):
        a, b = arc[max(i - 1, 0)], arc[min(i + 1, len(arc) - 1)]
        t = V(b[0] - a[0], -(b[1] - a[1]), 0).normalized()        # along the edge, sim xy
        nrm = V(-t.y, t.x, 0)                                      # outward (up / sideways)
        w = 0.0055 + 0.016 * (abs(x) / 450.0) ** 4
        p = pp(x, d)
        po, pi = p + nrm * 0.0015, p - nrm * w
        rings.append([po + V(0, 0, 0.004), po + V(0, 0, -0.016), pi + V(0, 0, -0.012), pi + V(0, 0, 0.004)])
    mb.loft(rings, "black")
    # glareshield: a padded hood from the panel top forward to the firewall. It is as wide
    # as the cabin, and ahead of the panel it follows the windshield down a few mm below
    # the glass, so it meets the walls and the windshield base with no slot: in VR you
    # cannot look past it into the space behind the panel, or from the footwell up
    # through the bottom of the windshield.
    mb.loft([glareshield_section(zz) for zz in GLARESHIELD_Z], "black")

    build_g3x(mb)

    # --- top row: engine start, slip ball, standby airspeed / vertical speed / altimeter
    led(mb, -249.7, 44.8, "lens_green", r=0.0024)
    c = pp(-226.8, 45.4)
    mb.disc(c + V(0, 0, -0.002), (0, 0, 1), 0.012, 0.004, n=20, mat="chrome")
    button = MB()
    button.disc(c + V(0, 0, -0.006), (0, 0, 1), 0.0078, 0.005, n=20, mat="metal")
    panel_part("BUTTON_Start", button, c, "slide", 1, [(0, 0.0), (100, -0.003)])   # Blender -Y = into the panel
    c = pp(-126.0, 44.3)
    mb.rbox(c + V(0, 0, -0.011), (0.067, 0.036, 0.022), 0.005, "black")
    tube = [c + V(0.026 * (k / 6.0 - 1), 0.004 * (1 - (k / 6.0 - 1) ** 2) - 0.002, -0.0225) for k in range(13)]
    mb.sweep(tube, radius=0.0045, n=10, mat="gauge_mark")
    mb.ellipsoid(c + V(0, -0.0015, -0.027), 0.0034, 0.0034, 0.0034, "black", nu=4, nv=10)
    gy = PANEL_TOP_Y - 0.063
    asi_arcs = ((asi_angle(35), asi_angle(81), "arc_white"), (asi_angle(43), asi_angle(94.5), "arc_green"),
                (asi_angle(94.5), asi_angle(124), "arc_yellow"), (asi_angle(124), asi_angle(126), "arc_red"))
    gauge(mb, -0.043, gy, 0.032, arcs=asi_arcs, ticks=11, span=(25, 330), needle_deg=2, bezel=0.0075)
    gauge(mb, 0.051, gy, 0.032, ticks=20, span=(-252, 90), needle_deg=-90, bezel=0.0075)
    gauge(mb, 0.145, gy, 0.032, ticks=10, span=(0, 324), needle_deg=210, bezel=0.0075)
    c = pp(112.0, 88.0)
    knob(mb, c.x, c.y, r=0.0075, depth=0.012)                    # altimeter baro knob
    plate(mb, 246.2, 50.5, 73.7, 23.5, lines=5)                  # passenger warning

    # --- left column: fuel reserve light, test, engine lanes A/B, fuel pumps, battery backup
    plate(mb, -387.2, 107.4, 31.0, 10.4, lines=2)
    plate(mb, -403.3, 120.8, 14.0, 4.4, lines=1)
    c = pp(-403.3, 129.0)
    mb.disc(c + V(0, 0, -0.002), (0, 0, 1), 0.0055, 0.004, n=6, mat="chrome")
    button = MB()
    button.disc(c + V(0, 0, -0.005), (0, 0, 1), 0.0035, 0.003, n=12, mat="knob_red")
    panel_part("BUTTON_Test", button, c, "slide", 1, [(0, 0.0), (100, -0.002)])
    led(mb, -387.2, 129.0)
    for row_d, lbl_d, off_d, (g0, g1), leds, what in ((180.3, 164.2, 195.6, (167.2, 197.3), True, "Lane"),
                                                      (221.3, 204.4, 236.6, (208.2, 238.3), False, "Pump")):
        for x, tag in ((-410.1, "A"), (-387.2, "B")):
            if leds:
                led(mb, x + 0.6, 155.7)
            plate(mb, x, lbl_d, 16.0, 5.0, lines=1)
            c = pp(x, row_d)
            toggle(mb, c.x, c.y, angle=-20, name="SWITCH_%s_%s" % (what, tag))
            plate(mb, x, off_d, 11.0, 4.5, lines=1)
        for gx in (-420.0, -398.1, -376.2):                    # switch guards
            mb.rbox(pp(gx, (g0 + g1) / 2, -0.010), (0.0042, (g1 - g0) / 1000, 0.020), 0.0019, "chrome")
    plate(mb, -435.0, 203.8, 15.0, 3.8, lines=1)
    # battery backup: a short toggle under a red guard that flips up on a hinge near its top
    # edge. The hinge block on the panel stays inside the guard whatever its angle, and the
    # axis sits at mid-depth so the guard never swings into the panel.
    c = pp(-435.0, 224.6)
    toggle(mb, c.x, c.y, angle=-20, name="SWITCH_Battery_Backup", length=0.007, ball=0.002)
    mb.rbox(pp(-435.0, 208.5, -0.004), (0.017, 0.005, 0.008), 0.001, "chrome")
    guard = MB()
    guard.rbox(pp(-435.0, 224.6, -0.006), (0.015, 0.038, 0.012), 0.0045, "knob_red")
    guard.rbox(pp(-435.0, 224.6, -0.0122), (0.007, 0.026, 0.0008), 0.0, "label")
    panel_part("SWITCH_Battery_Guard", guard, pp(-435.0, 208.5, -0.006), "rot", 0, [(0, 0), (100, 100)])

    # --- centre: COM radio, switch row, key switch, transponder
    mb.rbox(pp(17.2, 127.0, -0.004), (0.159, 0.0323, 0.008), 0.0015, "black")
    mb.rbox(pp(17.5, 125.35, -0.0082), (0.0883, 0.0219, 0.0006), 0.0, "lcd")
    for x, d in ((-54.8, 118.4), (-37.5, 118.4), (-37.5, 130.8), (72.7, 118.4), (90.9, 118.4)):
        mb.rbox(pp(x, d, -0.0095), (0.010, 0.006, 0.003), 0.0012, "button")
    mb.disc(pp(-46.1, 124.4, -0.0088), (0, 0, 1), 0.0028, 0.0016, n=10, mat="metal")
    c = pp(-55.7, 130.8)
    knob(mb, c.x, c.y, r=0.006, depth=0.012, z=z - 0.008)
    c = pp(85.4, 132.6)
    knob(mb, c.x, c.y, r=0.0085, depth=0.010, z=z - 0.008)
    knob(mb, c.x, c.y, r=0.0055, depth=0.017, z=z - 0.008)
    for i, name in enumerate(ROW_SWITCHES):
        panel_switch(mb, -71.2 + i * 21.96, 160.0, "SWITCH_" + name)
    c = pp(-55.7, 200.0)
    mb.disc(c + V(0, 0, -0.002), (0, 0, 1), 0.0127, 0.004, n=20, mat="chrome")
    key = MB()                  # engine / avionics key: lock barrel and key slot turn together
    key.tube(c + V(0, 0, -0.0035), c + V(0, 0, -0.011), 0.0085, n=16, mat="metal")
    key.rbox(c + V(0, 0, -0.0115), (0.0016, 0.0085, 0.002), 0.0, "black")
    panel_part("KEY_Avionics", key, c, "rot", 1, [(0, 0), (100, -90)])   # -Y about Blender Y = clockwise
    plate(mb, -73.0, 195.9, 23.7, 17.3, lines=3)
    # transponder control head (Trig TT21 style)
    tx, td = 62.65, 200.0
    mb.rbox(pp(tx, td, -0.004), (0.0619, 0.0438, 0.008), 0.004, "black")
    mb.rbox(pp(61.7, 195.4, -0.0082), (0.031, 0.0164, 0.0006), 0.0, "lcd_light")
    for x, d in ((39.0, 190.8), (86.3, 190.8), (86.3, 203.6)):
        mb.disc(pp(x, d, -0.0095), (0, 0, 1), 0.0035, 0.003, n=12, mat="gauge_mark")
    mb.rbox(pp(60.8, 214.6, -0.0095), (0.010, 0.005, 0.003), 0.0024, "gauge_mark")
    for x in (44.4, 79.1):
        c = pp(x, 214.6)
        knob(mb, c.x, c.y, r=0.0058, depth=0.010, z=z - 0.008)
    for dx in (-27.0, 27.0):
        for dd in (-18.5, 18.5):
            screw(mb, tx + dx, td + dd, -0.008)
    led(mb, 103.7, 181.7, r=0.0038)

    # --- right: placards, 12 V and USB sockets, circuit breakers, map box
    plate(mb, 158.2, 132.6, 69.4, 42.1, lines=8)               # pre-flight check
    plate(mb, 244.3, 132.6, 97.3, 42.1, lines=7)               # Rotax 915 iS limits
    plate(mb, 368.3, 91.5, 24.0, 10.4, lines=2)
    c = pp(367.8, 111.2)
    mb.disc(c + V(0, 0, -0.003), (0, 0, 1), 0.0135, 0.006, n=24, mat="chrome")
    mb.disc(c + V(0, 0, -0.0062), (0, 0, 1), 0.0090, 0.0006, n=20, mat="inlet_dark")
    c = pp(368.3, 152.2)
    mb.disc(c + V(0, 0, -0.003), (0, 0, 1), 0.0175, 0.006, n=24, mat="black")
    for dd in (-4.8, 4.8):
        mb.rbox(pp(368.3, 152.2 + dd, -0.0062), (0.0085, 0.0032, 0.0006), 0.0, "inlet_dark")
    plate(mb, 368.3, 175.1, 10.0, 5.0, lines=1)
    rows = (113.1, 152.2, 167.8, 183.6, 199.2, 214.8, 230.9)
    for x, lx, ds in ((402.7, 390.2, rows), (421.0, 431.7, rows[1:])):
        for d in ds:
            c = pp(x, d)
            mb.disc(c + V(0, 0, -0.0015), (0, 0, 1), 0.006, 0.003, n=16, mat="black")
            mb.tube(c + V(0, 0, -0.003), c + V(0, 0, -0.008), 0.0034, n=12, mat="button")
            plate(mb, lx, d, 8.0, 4.5, lines=1)
    box = rounded_poly([(133.9, 180.9), (348.6, 180.9), (383.0, 205.5), (383.0, 261.2), (133.9, 261.2)], 11.0)
    flat_poly(mb, box, z - 0.0025, z, "black")
    inner = rounded_poly([(140.5, 187.5), (345.6, 187.5), (376.4, 209.5), (376.4, 254.6), (140.5, 254.6)], 7.0)
    flat_poly(mb, inner, z - 0.0027, z - 0.0024, "inlet_dark")
    for x, d in ((-416.4, 257.4), (414.2, 254.6), (-85.8, 257.4), (119.1, 257.4)):
        screw(mb, x, d)

    # --- console plate below: vernier throttle, cabin heat, brake lever
    tab = rounded_poly([(-77.0, 228.0), (111.0, 228.0), (111.0, 282.0), (105.0, 301.6), (-71.0, 301.6),
                        (-77.0, 282.0)], 8.0)
    flat_poly(mb, tab, z - 0.005, z + 0.001, "panel_grey")
    tz = z - 0.005
    c = throttle_axis()
    mb.disc(V(c.x, c.y, tz - 0.005), (0, 0, 1), 0.0285, 0.010, n=8, mat="black")          # friction lock nut
    plate(mb, -36.1, 239.9, 37.0, 9.0, lines=2, dz=-0.005)          # the knob: build_throttle_knob()
    c = pp(70.0, 290.0)
    knob(mb, c.x, c.y, r=0.009, depth=0.020, z=tz)                                     # cabin heat
    plate(mb, 69.4, 272.2, 29.0, 8.0, lines=2, dz=-0.005)
    c = pp(-54.6, 252.0)
    mb.disc(V(c.x, c.y, tz - 0.003), (0, 0, 1), 0.006, 0.006, n=12, mat="metal")         # brake lever pivot
    mb.rbox(pp(-54.6, 289.4, -0.005 - 0.009), (0.0186, 0.079, 0.012), 0.0062, "knob_yellow")

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
    pivot = V(cx, -0.40, -0.45)          # the back's lower edge rests on the cushion's rear
    back_c = pivot + tilt @ V(0, 0.33, -0.05)
    mb.rbox(back_c, (0.40, 0.64, 0.07), 0.025, "seat", rot=tilt)
    mb.rbox(back_c + tilt @ V(0, -0.02, 0.04), (0.33, 0.54, 0.03), 0.014, "seat_fabric", rot=tilt)
    for dx in (-0.15, 0.15):
        mb.rbox(back_c + tilt @ V(dx, -0.04, 0.045), (0.05, 0.48, 0.05), 0.02, "seat", rot=tilt)
    # four-point harness lying on the seat, with buckle
    strap_profile = [(0, -0.022), (0.003, -0.022), (0.003, 0.022), (0, 0.022)]

    def strap(pts, ups):
        """Sweep a strap, splitting each run in four so its faces stay flat where it bends
        and twists (large warped faces cross over each other and open holes)."""
        P, U = [], []
        for i in range(len(pts) - 1):
            for k in range(4):
                P.append(pts[i].lerp(pts[i + 1], k / 4))
                U.append(ups[i].lerp(ups[i + 1], k / 4).normalized())
        mb.sweep(P + [pts[-1]], mat="belt", ups=U + [ups[-1]], profile=strap_profile)
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
        strap(pts, ups)
    for dx in (-0.2, 0.2):
        strap([V(cx + dx, -0.40, -0.40), V(cx + dx * 0.8, -0.35, -0.32), buckle + V(dx * 0.15, 0, 0)],
              [V(0, 1, 0)] * 3)
    mb.rbox(buckle + V(0, 0.004, 0), (0.06, 0.008, 0.06), 0.006, "chrome")
    # pedestal under the cushion and a support under the seat back, down into the carpet,
    # so the seat stands on the floor instead of hovering over it
    mb.rbox(V(cx, -0.465, -0.26), (0.32, 0.05, 0.40), 0.008, "seat")
    mb.rbox(V(cx, -0.445, -0.49), (0.30, 0.10, 0.05), 0.008, "seat")


# ============================================================================
# moving cockpit controls
# ============================================================================
# Each control is its own node with its origin on its real pivot, keyed 0..100 like the
# control surfaces. model/A240_Interior_Behavior.xml drives the keys from the sim:
# 50 is neutral for the stick, pedals and trim; 0 is idle, flaps up and brakes off.
# Every moving part stays inside what it is mounted on over its whole travel (the stick
# in its boot, the throttle shaft in its nut, the levers in the console), so no gap opens.
STICK_PITCH = (12.0, -16.0)   # stick angle at keys 0 (full forward) and 100 (full aft), deg
STICK_ROLL = 14.0             # either way, deg
PEDAL_TRAVEL = 18.0           # each pedal about its floor hinge, deg
TOE_BRAKE = 12.0              # pedal face tilt at full brake, deg
THROTTLE_PULL = 0.05          # knob pulled out this far at idle ("idle PULL"), m
FLAP_PIVOT = V(0.0, -0.43, -0.27)
FLAP_LEVER_UP = 30.0          # lever elevation with flaps up, deg (it leans forward)
FLAP_LEVER_TRAVEL = 45.0      # raised this far for full flaps, deg
TRIM_TRAVEL = 20.0            # trim lever either way, deg
SEATS = (("Pilot", -0.26), ("Copilot", 0.26))


def throttle_axis():
    """Where the throttle shaft meets the console plate (sim frame)."""
    c = pp(12.6, 252.0)
    return V(c.x, c.y, PANEL_Z - 0.005)


def build_control_mounts(mb, cx):
    """The fixed parts the moving controls sit in: a low rubber boot round the stick (its
    top is wide enough that the stick always comes out through it) and the pedal cross
    tube on the floor."""
    base = V(cx, -0.485, 0.14)
    mb.tube(base - V(0, 0.002, 0), base + V(0, 0.04, 0), 0.05, 0.03, n=16, mat="rubber")
    mb.tube(V(cx - 0.12, -0.475, 0.70), V(cx + 0.12, -0.475, 0.70), 0.009, n=8, mat="metal")


def build_stick(col, parent, seat, cx):
    """Control stick pivoting at the floor: an outer node rolls, the inner one pitches."""
    mb = MB()
    base = V(cx, -0.485, 0.14)
    top = V(cx, -0.21, 0.185)
    mb.tube(base, top, 0.011, n=10, mat="metal")
    grip0, grip1 = top, top + V(0, 0.115, 0.012)
    mb.tube(grip0, grip1, 0.017, 0.019, n=16, mat="black")
    for i in range(1, 4):
        p = grip0 + (grip1 - grip0) * (i / 4)
        mb.sweep([p + V(0.0195 * math.cos(a), 0, 0.0195 * math.sin(a)) for a in
                  [2 * math.pi * j / 16 for j in range(16)]], radius=0.0025, n=5, mat="black", closed=True)
    mb.ellipsoid(grip1 + V(0, 0.004, 0), 0.02, 0.012, 0.02, "black", nu=6, nv=14)
    mb.ellipsoid(grip1 + V(0, 0.012, 0.004), 0.005, 0.004, 0.005, "knob_red", nu=4, nv=8)
    mb.rbox(grip1 + V(0, -0.01, -0.016), (0.012, 0.012, 0.008), 0.002, "black")
    roll = make_empty("HANDLING_Stick_Roll_%s" % seat, col, parent, base, size=0.05)
    pitch = make_object("HANDLING_Stick_Pitch_%s" % seat, mb, col, roll, pivot=base, smooth=40)
    # Blender X = sim -x and Blender Y = sim -z: +X tips the stick forward, +Y to the left
    fwd, aft = STICK_PITCH
    animate(roll, 1, [(0, STICK_ROLL), (50, 0), (100, -STICK_ROLL)], "stick_roll_%s" % seat.lower())
    animate(pitch, 0, [(0, fwd), (50, 0), (100, aft)], "stick_pitch_%s" % seat.lower())


def build_pedals(col, parent, seat, cx):
    """Rudder pedals hinged at the floor; each pedal face tilts on its arm for the toe brake."""
    for tag, dx in (("L", -0.085), ("R", 0.085)):
        x = cx + dx
        piv = V(x, -0.475, 0.70)
        top = V(x, -0.38, 0.725)
        arm = MB()
        arm.tube(piv, top, 0.008, n=8, mat="metal")
        face = MB()
        pedal, tilt = top + V(0, 0.01, -0.004), rot_x(-20)
        face.rbox(pedal, (0.065, 0.11, 0.014), 0.006, "black", rot=tilt)
        for k in range(4):            # grip ribs, set into the pedal's face
            face.rbox(pedal + tilt @ V(0, -0.045 + k * 0.024, -0.0085), (0.06, 0.004, 0.004), 0.0012, "metal",
                      rot=tilt)
        a = make_object("HANDLING_Pedal_%s_%s" % (tag, seat), arm, col, parent, pivot=piv, smooth=40)
        f = make_object("HANDLING_ToeBrake_%s_%s" % (tag, seat), face, col, a, pivot=top, smooth=40)
        # +X tips the top forward. Key 100 = right pedal pushed in, so the left one comes back.
        k = PEDAL_TRAVEL if tag == "R" else -PEDAL_TRAVEL
        animate(a, 0, [(0, -k), (50, 0), (100, k)], "rudder_pedal_%s_%s" % (tag.lower(), seat.lower()))
        animate(f, 0, [(0, 0), (100, TOE_BRAKE)], "toe_brake_%s_%s" % (tag.lower(), seat.lower()))


def build_throttle_knob(col, parent):
    """Vernier push-pull throttle, modelled pushed fully in (full power). Its shaft runs back
    into the panel backing, and is still inside the friction-lock nut when pulled to idle."""
    c = throttle_axis()
    mb = MB()

    def at(dz):
        return V(c.x, c.y, c.z + dz)
    mb.tube(at(0.044), at(-0.014), 0.005, n=12, mat="chrome")
    mb.tube(at(-0.014), at(-0.040), 0.019, 0.0185, n=24, mat="black")
    mb.sweep([V(c.x + 0.0195 * math.cos(a), c.y + 0.0195 * math.sin(a), c.z - 0.027) for a in
              [2 * math.pi * j / 24 for j in range(24)]], radius=0.002, n=5, mat="black", closed=True)
    mb.tube(at(-0.040), at(-0.045), 0.010, 0.009, n=16, mat="button")
    obj = make_object("THROTTLE_Knob", mb, col, parent, pivot=c, smooth=40)
    animate_slide(obj, 1, [(0, THROTTLE_PULL), (100, 0.0)], "throttle_knob")   # Blender +Y = towards the pilot


def build_console_levers(col, parent):
    """Flap lever (sweeping its notched quadrant) and the elevator trim lever beside it."""
    a = math.radians(FLAP_LEVER_UP)
    d = V(0, math.sin(a), math.cos(a))
    mb = MB()
    mb.tube(FLAP_PIVOT, FLAP_PIVOT + d * 0.178, 0.008, n=8, mat="metal")
    mb.tube(FLAP_PIVOT + d * 0.178, FLAP_PIVOT + d * 0.228, 0.016, 0.014, n=12, mat="knob_red")
    flap = make_object("HANDLING_Flap_Lever", mb, col, parent, pivot=FLAP_PIVOT, smooth=40)
    # -X raises a forward-leaning lever: up for flaps
    animate(flap, 0, [(0, 0), (100, -FLAP_LEVER_TRAVEL)], "flap_lever")
    piv = V(0.035, -0.44, -0.30)
    mb = MB()
    mb.tube(piv, V(0.035, -0.36, -0.25), 0.005, n=8, mat="metal")
    mb.ellipsoid((0.035, -0.355, -0.247), 0.01, 0.012, 0.018, "black", nu=4, nv=8)
    trim = make_object("HANDLING_Trim_Lever", mb, col, parent, pivot=piv, smooth=40)
    animate(trim, 0, [(0, TRIM_TRAVEL), (50, 0), (100, -TRIM_TRAVEL)], "elevator_trim_lever")   # 100 = nose up, lever aft


def build_moving_controls(col, parent):
    for seat, cx in SEATS:
        build_stick(col, parent, seat, cx)
        build_pedals(col, parent, seat, cx)
    build_throttle_knob(col, parent)
    build_console_levers(col, parent)


def frame_point(z, phi_deg, radius):
    """Centre of a cage tube of `radius` running along the skin at (z, phi): its outside
    3 mm in from the skin, measured along the skin's normal (which matters where the
    windshield slopes steeply), so the tube is set into the cabin lining - mounted on the
    wall rather than floating in front of it - and never shows through the skin."""
    return fus_point(z, phi_deg) - skin_normal(z, phi_deg) * (radius + 0.003)


def build_frame(mb):
    """Visible steel tube cage: roof bows, door frames, sills, A-pillars."""
    def bar(pts, radius=0.012):
        mb.sweep([frame_point(z, a, radius) for z, a in pts], radius=radius, n=10, mat="frame")
    bar([(0.285, a) for a in range(18, 163, 6)])
    bar([(-0.62, a) for a in range(54, 127, 6)])
    for side in (-1, 1):
        def ph(a):
            return a if side > 0 else 180 - a
        bar([(z, ph(47)) for z in [0.79 - i * 0.049 for i in range(11)]])
        bar([(z, ph(52.5)) for z in [0.29 - i * 0.07 for i in range(17)]])
        bar([(z, ph(-25)) for z in [0.78 - i * 0.07 for i in range(24)]], radius=0.014)
        bar([(-0.86, a) for a in [ph(a) for a in range(-40, 58, 7)]])
        # inside door handle: a grip on two feet that stand on the door's plexiglass
        fr = skin_frame(-0.62, ph(-10))
        mb.rbox(fus_point(-0.62, ph(-10)) - fr.col[0] * 0.020, (0.012, 0.022, 0.12), 0.005, "black", rot=fr)
        for zf in (-0.572, -0.668):
            ff = skin_frame(zf, ph(-10))
            mb.rbox(fus_point(zf, ph(-10)) - ff.col[0] * 0.0102, (0.0185, 0.016, 0.014), 0.003, "black", rot=ff)


def build_cabin_interior(col, parent):
    mb = MB()
    bulkhead(mb, 0.90, 0.88, "panel")                   # firewall
    bulkhead(mb, -0.94, -0.96, "liner")                 # aft bulkhead
    # carpeted floor that follows the cabin width, rising into the footwell. Its top edges
    # run just past the cabin lining and its ends into the bulkheads, so no crack shows at
    # the walls; its underside is narrower and thins out where the belly rises towards the
    # tail, so it stays inside the skin.
    z0, z1 = LINER_Z
    zs = sorted({z0, z1, 0.62} | {z for z in fuselage_stations() if z0 < z < z1})
    rings = []
    for z in zs:
        y = -0.485 if z < 0.62 else -0.485 + (z - 0.62) * 0.12
        cy, yb = fus_center(z).y, fuselage_params(z)[1]
        yu = max(y - 0.015, cy + 0.985 * (yb - cy))
        wt, wb = wall_x(z, y), ring_halfwidth(z, yu, LINER_SCALE) - 0.002
        rings.append([V(-wt, y, z), V(wt, y, z), V(wb, yu, z), V(-wb, yu, z)])
    mb.loft(rings, "carpet")
    # centre console with fuel valve and park brake
    mb.rbox((0, -0.462, -0.05), (0.10, 0.05, 0.72), 0.012, "panel")
    mb.disc((0, -0.436, 0.16), (0, 1, 0), 0.02, 0.008, n=16, mat="black")
    mb.rbox((0, -0.428, 0.16), (0.008, 0.012, 0.05), 0.003, "knob_red")
    mb.rbox((0, -0.436, 0.16), (0.075, 0.001, 0.075), 0.0, "placard", rot=rot_y(45))
    mb.tube(V(0.0, -0.437, 0.25), V(0.0, -0.37, 0.30), 0.006, n=8, mat="metal")
    mb.ellipsoid((0.0, -0.365, 0.303), 0.012, 0.012, 0.02, "black", nu=4, nv=8)
    # flap lever quadrant, one notch per flap setting (the levers: build_moving_controls)
    mb.rbox((0, -0.415, -0.22), (0.012, 0.10, 0.16), 0.004, "metal")
    for i in range(3):
        a = math.radians(FLAP_LEVER_UP + i * FLAP_LEVER_TRAVEL / 2)
        mb.rbox(FLAP_PIVOT + V(0, 0.06 * math.sin(a), 0.06 * math.cos(a)), (0.014, 0.006, 0.006), 0.001, "black")
    # baggage shelf and its front lip, wall to wall (the cabin tapers here, so their sides
    # follow it) and into the aft bulkhead; and a fire extinguisher behind the seats
    def wall_slab(z0, z1, y0, y1, mat):
        mb.loft([[V(-wall_x(zz, y1), y1, zz), V(wall_x(zz, y1), y1, zz), V(wall_x(zz, y0), y0, zz),
                  V(-wall_x(zz, y0), y0, zz)] for zz in (z0, z1)], mat)
    wall_slab(-0.945, -0.63, -0.32, -0.30, "carpet")
    wall_slab(-0.641, -0.629, -0.305, -0.235, "panel")
    mb.tube(V(0.30, -0.30, -0.80), V(0.30, -0.10, -0.80), 0.035, n=16, mat="knob_red")
    mb.tube(V(0.30, -0.10, -0.80), V(0.30, -0.075, -0.80), 0.014, n=10, mat="black")
    mb.tube(V(0.30, -0.08, -0.80), V(0.30, -0.08, -0.74), 0.005, n=6, mat="black")
    for y in (-0.26, -0.16):
        mb.sweep([V(0.30 + 0.039 * math.cos(a), y, -0.80 + 0.039 * math.sin(a)) for a in
                  [2 * math.pi * j / 16 for j in range(16)]], radius=0.004, n=5, mat="metal", closed=True)
    # fuel sight gauges hanging from the roof at the wing roots (the tanks are above them),
    # beside the skylight; the top fitting is set into the lining
    for side in (-1, 1):
        x, z = side * 0.40, 0.05
        slope = (lining_y(x + 0.005, z) - lining_y(x - 0.005, z)) / 0.01
        up = V(-slope, 1, 0).normalized()                         # ceiling normal here
        top = V(x, lining_y(x, z), z) - up * 0.005                # fitting 2 mm into the lining
        mb.tube(V(x, 0.45, z), top, 0.008, n=10, mat="glass")
        mb.tube(V(x, 0.45, z), V(x, 0.54, z), 0.0065, n=8, mat="placard")       # fuel
        mb.rbox((x, 0.45, z), (0.024, 0.014, 0.03), 0.004, "metal")
        mb.rbox(top, (0.024, 0.014, 0.03), 0.004, "metal", rot=rot_z(math.degrees(math.atan(slope))))
    build_panel(mb)
    build_frame(mb)
    for side in (-1, 1):
        cx = side * 0.26
        build_seat(mb, cx)
        build_control_mounts(mb, cx)
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


def animate_slide(obj, axis_index, keys, name):
    """Location keys (frame, metres along a Blender axis from where it was built), linear."""
    base = obj.location[axis_index]
    for frame, offset in keys:
        obj.location[axis_index] = base + offset
        obj.keyframe_insert("location", index=axis_index, frame=frame)
    act = obj.animation_data.action
    act.name = name
    for fc in _fcurves(act):
        for kp in fc.keyframe_points:
            kp.interpolation = "LINEAR"
    obj.location[axis_index] = base


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
    build_moving_controls(inn, iroot)
    build_panel_controls(inn, iroot)
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
