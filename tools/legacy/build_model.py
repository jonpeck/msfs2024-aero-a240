#!/usr/bin/env python3
"""LEGACY - the original procedural A240 model (flat-shaded, no Blender).

Superseded by blender/A240.blend, which is exported by tools/blender/export_a240.py.
Kept for reference only: running it OVERWRITES the exported A240.gltf / A240_interior.gltf
and the previews with the old low-detail model.

Generate the Aero A240 3D models (exterior + interior glTF) and preview images.

Pure standard library - no Blender, no third-party packages.

Authoring frame ("sim frame"), metres:  +x = right wing, +y = up, +z = nose.
Origin: centreline, wing quarter-chord, propeller thrust-line height.
This matches the datum used by flight_model.cfg (which is in feet).

glTF frame is right-handed with +Y up and +Z forward, so +X is the LEFT wing:
every point is written as (-x, y, z) and triangle winding is fixed afterwards.

Two models are written, following the MSFS 2024 SDK sample aircraft:
  A240.gltf           the airframe, shown in every view
  A240_interior.gltf  cabin lining, panel, seats - drawn together with the exterior
The output is glTF *source* for the SDK package builder (see tools/build_sdk.ps1).

Usage:  py tools/build_model.py [--no-preview] [--strict]
"""
import json
import math
import os
import struct
import sys
import zlib

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AIRCRAFT_DIR = os.path.join(ROOT, "sdk-project", "PackageSources", "SimObjects", "Airplanes", "Local_A240")
MODEL_DIR = os.path.join(AIRCRAFT_DIR, "common", "model")
THUMBNAIL_DIR = os.path.join(AIRCRAFT_DIR, "presets", "local", "A240", "thumbnail")
PREVIEW_DIR = os.path.join(ROOT, "previews")

SCREEN_MATERIAL = "EFIS_Display"   # must match "texture=" in common/panel/panel.cfg
SKIP = "__skip__"                  # faces generated only to orient a shell, then dropped

# name: sRGB colour, alpha, metallic, roughness, double sided, optional Asobo extensions
def _m(r, g, b, a=1.0, metal=0.0, rough=0.5, double=False, ext=None, emissive=None):
    return dict(rgb=(r, g, b), a=a, metal=metal, rough=rough, double=double, ext=ext, emissive=emissive)

GLASS_EXT = {"ASOBO_material_draw_order": {"drawOrderOffset": 2},
             "ASOBO_material_glass_v2": {"glassWidth": 0.005}}
MATERIALS = {
    "white":  _m(0.93, 0.93, 0.91, rough=0.45),
    "red":    _m(0.74, 0.06, 0.07, rough=0.40),
    # Glazing. The sim lights a blended glass surface in proportion to its alpha, so a high
    # alpha washes out to white in direct sun (0.18 did; the pilot looks through two faces,
    # since it is double-sided). Values follow the SDK's DA62 sample: alpha 0.01-0.05,
    # neutral colour, smooth, 5 mm glass width.
    "glass":  _m(0.86, 0.86, 0.85, a=0.03, rough=0.05, double=True, ext=GLASS_EXT),
    "black":  _m(0.04, 0.04, 0.04, rough=0.60),
    "tire":   _m(0.03, 0.03, 0.03, rough=0.90),
    "panel":  _m(0.33, 0.34, 0.36, rough=0.70),
    "liner":  _m(0.66, 0.67, 0.68, rough=0.80),
    "floor":  _m(0.22, 0.22, 0.23, rough=0.85),
    "seat":   _m(0.28, 0.09, 0.09, rough=0.85),
    "blur":   _m(0.10, 0.10, 0.10, a=0.16, rough=0.50, double=True,
                 ext={"ASOBO_material_draw_order": {"drawOrderOffset": 3}}),
    # The cockpit display: the sim paints the HTML instrument onto the material with this name.
    SCREEN_MATERIAL: _m(0.32, 0.32, 0.32, rough=0.80, emissive=5000.0,
                        ext={"ASOBO_material_emissive": {"emissiveDayMultiplier": 1.0,
                                                         "emissiveNightMultiplier": 1.0}}),
}


# ----------------------------------------------------------------------------
# small vector helpers
# ----------------------------------------------------------------------------
def vadd(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def vsub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def vmul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def vdot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def vlen(a): return math.sqrt(vdot(a, a))
def vnorm(a):
    l = vlen(a)
    return (a[0] / l, a[1] / l, a[2] / l) if l > 1e-12 else (0.0, 0.0, 0.0)
def lerp(a, b, t): return a + (b - a) * t
def vlerp(a, b, t): return (lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t))
def centroid(pts):
    n = float(len(pts))
    return (sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n, sum(p[2] for p in pts) / n)


# ----------------------------------------------------------------------------
# mesh containers
# ----------------------------------------------------------------------------
class Part:
    """A triangle soup in world sim-frame coordinates.

    closed=True  -> a watertight, consistently wound solid; outward direction is
                    recovered from the signed volume.
    closed=False -> an open sheet; triangles are flipped to face `facing`.
    """
    def __init__(self, closed=True, facing=None, crease=40.0, invert=False):
        self.tris = []          # (p0, p1, p2, material, (uv0, uv1, uv2) or None)
        self.closed = closed
        self.facing = facing
        self.crease = crease
        self.invert = invert    # closed shells only: face inwards (a room, not a solid)

    def tri(self, a, b, c, mat, uv=None):
        self.tris.append((a, b, c, mat, uv))

    def quad(self, a, b, c, d, mat, uv=None):
        if uv:
            self.tri(a, b, c, mat, (uv[0], uv[1], uv[2]))
            self.tri(a, c, d, mat, (uv[0], uv[2], uv[3]))
        else:
            self.tri(a, b, c, mat)
            self.tri(a, c, d, mat)


class Node:
    def __init__(self, name, pivot=(0.0, 0.0, 0.0)):
        self.name = name
        self.pivot = pivot       # world sim-frame position of the node origin
        self.parts = []
        self.children = []
        self.index = None

    def add(self, *parts):
        self.parts.extend(parts)
        return self

    def child(self, node):
        self.children.append(node)
        return node


# ----------------------------------------------------------------------------
# primitives
# ----------------------------------------------------------------------------
def loft(rings, mat="white", mat_fn=None, cap_start=True, cap_end=True, crease=40.0):
    """Skin a sequence of rings (each the same number of points) into a closed solid."""
    part = Part(closed=True, crease=crease)
    n = len(rings[0])
    pick = (lambda pts: mat_fn(centroid(pts))) if mat_fn else (lambda pts: mat)
    for i in range(len(rings) - 1):
        a, b = rings[i], rings[i + 1]
        for j in range(n):
            k = (j + 1) % n
            quad = (a[j], a[k], b[k], b[j])
            part.quad(*quad, pick(quad))
    if cap_start:
        a = rings[0]
        c = centroid(a)
        for j in range(n):
            k = (j + 1) % n
            part.tri(c, a[k], a[j], pick((c, a[k], a[j])))
    if cap_end:
        b = rings[-1]
        c = centroid(b)
        for j in range(n):
            k = (j + 1) % n
            part.tri(c, b[j], b[k], pick((c, b[j], b[k])))
    return part


def box(x0, x1, y0, y1, z0, z1, mat):
    r0 = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)]
    r1 = [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    return loft([r0, r1], mat, crease=30.0)


def tube(p0, p1, r0, r1=None, n=10, mat="white", flat=1.0):
    """Round (or flattened) tube between two points."""
    r1 = r0 if r1 is None else r1
    axis = vnorm(vsub(p1, p0))
    ref = (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = vnorm(vcross(axis, ref))
    v = vcross(axis, u)
    rings = []
    for p, r in ((p0, r0), (p1, r1)):
        ring = []
        for j in range(n):
            a = 2 * math.pi * j / n
            ring.append(vadd(p, vadd(vmul(u, r * math.cos(a)), vmul(v, r * flat * math.sin(a)))))
        rings.append(ring)
    return loft(rings, mat, crease=50.0)


def ellipsoid(c, rx, ry, rz, mat, nu=10, nv=16):
    rings = []
    for i in range(nu + 1):
        th = math.radians(6 + (168.0 * i / nu))      # avoid degenerate poles
        z = c[2] + rz * math.cos(th)
        s = math.sin(th)
        rings.append([(c[0] + rx * s * math.cos(2 * math.pi * j / nv),
                       c[1] + ry * s * math.sin(2 * math.pi * j / nv), z) for j in range(nv)])
    return loft(rings, mat, crease=60.0)


def wheel(c, radius, width, mat="tire", n=20):
    """Tyre with its axle along x."""
    prof = [(-0.5, 0.62), (-0.42, 0.90), (-0.2, 1.0), (0.2, 1.0), (0.42, 0.90), (0.5, 0.62)]
    rings = []
    for fx, fr in prof:
        rings.append([(c[0] + fx * width, c[1] + radius * fr * math.cos(2 * math.pi * j / n),
                       c[2] + radius * fr * math.sin(2 * math.pi * j / n)) for j in range(n)])
    return loft(rings, mat, crease=50.0)


def airfoil_loop(m, p, t, x0=0.0, x1=1.0, n=10):
    """NACA 4-digit section between chord fractions x0..x1 as a closed loop of (xc, yc)."""
    def thick(x):
        return 5 * t * (0.2969 * math.sqrt(max(x, 0.0)) - 0.1260 * x - 0.3516 * x ** 2
                        + 0.2843 * x ** 3 - 0.1036 * x ** 4)
    def camber(x):
        if m == 0 or p == 0:
            return 0.0
        if x < p:
            return m / p ** 2 * (2 * p * x - x * x)
        return m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x * x)
    xs = [x0 + (x1 - x0) * 0.5 * (1 - math.cos(math.pi * i / n)) for i in range(n + 1)]
    up = [(x, camber(x) + thick(x)) for x in xs]
    lo = [(x, camber(x) - thick(x)) for x in xs]
    if x0 <= 1e-9:
        lo = lo[1:]            # shared leading-edge point
    if x1 >= 1 - 1e-9:
        lo = lo[:-1]           # shared trailing-edge point
    return up[::-1] + lo


def section(origin, chord, chord_dir, thick_dir, loop):
    return [vadd(origin, vadd(vmul(chord_dir, xc * chord), vmul(thick_dir, yc * chord)))
            for xc, yc in loop]


# ----------------------------------------------------------------------------
# aircraft geometry
# ----------------------------------------------------------------------------
GROUND_Y = -1.12

# z, half width, y bottom, y top, superellipse exponent
FUSELAGE = [
    (1.80, 0.20, -0.24, 0.19, 2.4),     # cowl front
    (1.62, 0.31, -0.35, 0.25, 2.5),
    (1.25, 0.41, -0.45, 0.30, 2.8),
    (0.95, 0.49, -0.52, 0.32, 3.2),     # firewall
    (0.80, 0.53, -0.55, 0.34, 3.5),     # windshield base
    (0.31, 0.56, -0.58, 0.66, 4.0),     # windshield top / wing leading edge
    (0.00, 0.57, -0.58, 0.66, 4.0),
    (-0.45, 0.56, -0.56, 0.66, 4.0),
    (-0.94, 0.50, -0.50, 0.62, 3.6),    # aft cabin / wing trailing edge
    (-1.40, 0.37, -0.38, 0.47, 3.0),
    (-2.20, 0.20, -0.20, 0.31, 2.6),
    (-3.00, 0.08, -0.03, 0.22, 2.2),
    (-3.25, 0.045, 0.02, 0.19, 2.0),    # tail post
]
FUSELAGE_BREAKS = [0.26, -0.60, -0.80, 1.55]   # extra ring positions for paint / glazing edges


def fuselage_params(z):
    s = FUSELAGE
    if z >= s[0][0]:
        return s[0][1:]
    for a, b in zip(s, s[1:]):
        if b[0] <= z <= a[0]:
            t = (a[0] - z) / (a[0] - b[0])
            return tuple(lerp(a[i], b[i], t) for i in range(1, 5))
    return s[-1][1:]


def fuselage_ring(z, n=32):
    hw, yb, yt, e = fuselage_params(z)
    cy, hh = (yb + yt) / 2, (yt - yb) / 2
    ring = []
    for j in range(n):
        phi = 2 * math.pi * j / n
        c, s = math.cos(phi), math.sin(phi)
        r = (abs(c / hw) ** e + abs(s / hh) ** e) ** (-1.0 / e)
        ring.append((r * c, cy + r * s, z))
    return ring


def fuselage_material(c):
    """Paint and glazing, chosen per skin panel.

    Panels are picked by their polar angle around the ring centre (0 = side, 90 = top),
    so every edge follows a row of the mesh and stays straight.
    """
    x, y, z = c
    hw, yb, yt, _ = fuselage_params(z)
    phi = math.degrees(math.atan2(y - (yb + yt) / 2, abs(x)))
    if 0.31 < z < 0.80 and phi > 45:
        return "glass"                                   # windshield
    if -0.80 < z < 0.26 and -22 < phi < 45:
        return "glass"                                   # door glazing
    if -0.60 < z < 0.26 and phi > 56:
        return "glass"                                   # skylight
    if z > 1.55:
        return "red"                                     # cowl nose
    if -34 < phi < -22:
        return "red"                                     # side stripe
    return "white"


def fuselage_stations():
    keys = sorted(set([s[0] for s in FUSELAGE] + FUSELAGE_BREAKS), reverse=True)
    zs = []
    for a, b in zip(keys, keys[1:]):
        steps = max(1, int(math.ceil((a - b) / 0.11)))
        zs.extend(a + (b - a) * i / steps for i in range(steps))
    zs.append(keys[-1])
    return zs


def build_fuselage():
    return loft([fuselage_ring(z) for z in fuselage_stations()], mat_fn=fuselage_material, crease=35.0)


CABIN_FRONT_Z, CABIN_REAR_Z = 0.95, -0.94     # firewall and aft bulkhead stations


def build_liner():
    """Cabin lining: the fuselage skin between the bulkheads, 4 mm inside it, facing inwards.

    The exterior skin is single-sided, so without this the pilot would see straight
    through the cabin walls. Glazed panels are left open.
    """
    rings = []
    for z in fuselage_stations():
        if CABIN_REAR_Z - 1e-6 <= z <= CABIN_FRONT_Z + 1e-6:
            ring = fuselage_ring(z)
            c = centroid(ring)
            rings.append([(c[0] + (p[0] - c[0]) * 0.992, c[1] + (p[1] - c[1]) * 0.992, z) for p in ring])
    part = loft(rings, mat_fn=lambda c: SKIP if fuselage_material(c) == "glass" else "liner", crease=35.0)
    n = len(rings[0])
    part.tris = [(a, b, c, SKIP if i >= (len(rings) - 1) * n * 2 else m, uv)   # drop the end caps
                 for i, (a, b, c, m, uv) in enumerate(part.tris)]
    part.invert = True
    return part


WING_Y = 0.60
WING_LE_Z = 0.31
WING_CHORD = 1.02
WING_ROOT_X = 0.50
WING_TIP_X = 4.56
WING_INCIDENCE = math.radians(2.0)
FLAPERON_PIVOT = (0.0, 0.50, -0.72)


def build_wing(side):
    loop = airfoil_loop(0.04, 0.4, 0.13, n=12)
    cd = (0.0, -math.sin(WING_INCIDENCE), -math.cos(WING_INCIDENCE))
    td = (0.0, math.cos(WING_INCIDENCE), -math.sin(WING_INCIDENCE))
    xs = [0.50, 1.2, 2.0, 2.8, 3.5, 4.06, 4.40, 4.56]
    rings = [section((side * x, WING_Y, WING_LE_Z), WING_CHORD, cd, td, loop) for x in xs]
    # slightly tapered, rounded tip
    tip = section((side * 4.63, WING_Y + 0.01, WING_LE_Z - 0.10), WING_CHORD * 0.78, cd, td, loop)
    rings.append(tip)
    return loft(rings, mat_fn=lambda c: "red" if abs(c[0]) > 4.06 else "white", crease=40.0)


def build_flaperon(side):
    loop = airfoil_loop(0.0, 0.0, 0.14, n=6)
    xs = [0.62, 1.6, 2.6, 3.6, 4.50]
    rings = [section((side * x, 0.50, -0.70), 0.23, (0, 0, -1), (0, 1, 0), loop) for x in xs]
    return loft(rings, mat_fn=lambda c: "red" if abs(c[0]) > 4.06 else "white", crease=40.0)


def build_flaperon_brackets(side):
    return [box(side * x - 0.012, side * x + 0.012, 0.485, 0.575, -0.75, -0.60, "white")
            for x in (0.9, 2.1, 3.3, 4.3)]


STAB_Y = 0.10
ELEV_HINGE_Z = -3.20
RUDDER_HINGE_Z = -3.25


def tail_station(le_z, te_z, hinge_z):
    chord = le_z - te_z
    return chord, (le_z - hinge_z) / chord


def build_stabilizer():
    rings = []
    for x, le, te in ((-1.25, -2.86, -3.46), (0.0, -2.68, -3.52), (1.25, -2.86, -3.46)):
        chord, f = tail_station(le, te, ELEV_HINGE_Z)
        rings.append(section((x, STAB_Y, le), chord, (0, 0, -1), (0, 1, 0),
                             airfoil_loop(0, 0, 0.09, 0.0, f, n=8)))
    return loft(rings, "white", crease=40.0)


def build_elevator(side):
    rings = []
    for x, le, te in ((0.13, -2.70, -3.52), (1.25, -2.86, -3.46)):
        chord, f = tail_station(le, te, ELEV_HINGE_Z)
        rings.append(section((side * x, STAB_Y, le), chord, (0, 0, -1), (0, 1, 0),
                             airfoil_loop(0, 0, 0.09, f, 1.0, n=6)))
    return loft(rings, "white", crease=40.0)


def build_fin():
    rings = []
    for y, le, te in ((0.10, -2.55, -3.63), (0.70, -2.84, -3.57), (1.125, -3.05, -3.52)):
        chord, f = tail_station(le, te, RUDDER_HINGE_Z)
        rings.append(section((0, y, le), chord, (0, 0, -1), (1, 0, 0),
                             airfoil_loop(0, 0, 0.08, 0.0, f, n=8)))
    return loft(rings, mat_fn=lambda c: "red" if c[1] > 0.70 else "white", crease=40.0)


def build_rudder():
    rings = []
    for y, le, te in ((-0.02, -2.50, -3.65), (0.70, -2.84, -3.57), (1.125, -3.05, -3.52)):
        chord, f = tail_station(le, te, RUDDER_HINGE_Z)
        rings.append(section((0, y, le), chord, (0, 0, -1), (1, 0, 0),
                             airfoil_loop(0, 0, 0.08, f, 1.0, n=6)))
    return loft(rings, mat_fn=lambda c: "red" if c[1] > 0.70 else "white", crease=40.0)


def build_struts(side):
    foot = (side * 0.53, -0.46, -0.15)
    front = tube(foot, (side * 2.55, 0.555, 0.06), 0.030, n=10, flat=0.45)
    rear = tube(foot, (side * 2.55, 0.560, -0.52), 0.030, n=10, flat=0.45)
    jury = tube((side * 1.75, 0.16, -0.20), (side * 1.75, 0.555, -0.20), 0.012, n=6)
    return [front, rear, jury]


MAIN_X, MAIN_Z, MAIN_R = 1.00, -0.35, 0.19
NOSE_Z, NOSE_R = 1.25, 0.165


def build_gear():
    parts = []
    for side in (-1, 1):
        axle = (side * MAIN_X, GROUND_Y + MAIN_R, MAIN_Z)
        parts.append(tube((side * 0.32, -0.54, MAIN_Z), (side * (MAIN_X - 0.04), axle[1] + 0.04, MAIN_Z),
                          0.055, 0.04, n=10, flat=0.35))
        parts.append(wheel(axle, MAIN_R, 0.13))
        parts.append(ellipsoid((side * MAIN_X, axle[1] + 0.045, MAIN_Z - 0.03), 0.085, 0.165, 0.37, "red"))
    axle = (0.0, GROUND_Y + NOSE_R, NOSE_Z)
    parts.append(tube((0.0, -0.42, NOSE_Z + 0.06), (0.0, axle[1] + 0.05, NOSE_Z), 0.022, n=8))
    parts.append(wheel(axle, NOSE_R, 0.11))
    parts.append(ellipsoid((0.0, axle[1] + 0.04, NOSE_Z - 0.02), 0.075, 0.145, 0.31, "red"))
    return parts


PROP_Z = 1.86
PROP_R = 0.864        # 68 inch diameter


def build_spinner():
    rings = []
    z0, length, r0 = 1.805, 0.215, 0.115
    for t in (0.0, 0.2, 0.4, 0.6, 0.75, 0.87, 0.95, 0.99):
        r = r0 * math.sqrt(1 - t * t)
        rings.append([(r * math.cos(2 * math.pi * j / 16), r * math.sin(2 * math.pi * j / 16),
                       z0 + length * t) for j in range(16)])
    return loft(rings, "red", crease=60.0)


def build_blades():
    parts = []
    loop = airfoil_loop(0.02, 0.4, 0.11, n=5)
    stations = [(0.09, 0.060, 52), (0.22, 0.105, 40), (0.42, 0.115, 28),
                (0.62, 0.100, 21), (0.78, 0.075, 17.5), (0.864, 0.035, 16)]
    for b in range(3):
        psi = math.radians(90 + 120 * b)
        radial = (math.cos(psi), math.sin(psi), 0.0)
        tang = (-math.sin(psi), math.cos(psi), 0.0)
        rings = []
        for r, chord, beta in stations:
            be = math.radians(beta)
            cd = vadd(vmul(tang, math.cos(be)), (0.0, 0.0, -math.sin(be)))
            td = vadd(vmul(tang, math.sin(be)), (0.0, 0.0, math.cos(be)))
            origin = vsub(vadd((0.0, 0.0, PROP_Z), vmul(radial, r)), vmul(cd, chord * 0.4))
            rings.append(section(origin, chord, cd, td, loop))
        parts.append(loft(rings, "black", crease=50.0))
    return parts


def build_prop_blur():
    part = Part(closed=False, facing=(0, 0, 1))
    n = 40
    for j in range(n):
        a0, a1 = 2 * math.pi * j / n, 2 * math.pi * (j + 1) / n
        p = lambda r, a: (r * math.cos(a), r * math.sin(a), PROP_Z)
        part.quad(p(0.11, a0), p(PROP_R, a0), p(PROP_R, a1), p(0.11, a1), "blur")
    return part


SCREEN = dict(x0=-0.415, x1=-0.145, y0=0.000, y1=0.2025, z=0.489)   # 4:3, in front of the left seat


def bulkhead(z0, z1, mat="panel"):
    """A solid slice of the fuselage cross-section, shrunk a touch to sit inside the skin."""
    rings = []
    for z in (z0, z1):
        ring = fuselage_ring(z)
        c = centroid(ring)
        rings.append([(c[0] + (p[0] - c[0]) * 0.997, c[1] + (p[1] - c[1]) * 0.997, z) for p in ring])
    return loft(rings, mat, crease=30.0)


def build_cabin():
    # Everything here has to stay inside the fuselage skin (a rounded box about
    # 1.1 m wide in the cabin), so the boxes are kept clear of the corners.
    parts = [
        build_liner(),
        bulkhead(0.90, 0.88),                                    # firewall
        bulkhead(-0.94, -0.96),                                  # aft bulkhead
        box(-0.42, 0.42, -0.50, -0.485, -0.94, 0.56, "floor"),
        box(-0.52, 0.52, -0.10, 0.28, 0.50, 0.56, "panel"),      # instrument panel
        box(-0.45, 0.45, 0.28, 0.30, 0.48, 0.70, "black"),       # glareshield
        box(SCREEN["x0"] - 0.015, SCREEN["x1"] + 0.015, SCREEN["y0"] - 0.015, SCREEN["y1"] + 0.015,
            0.490, 0.50, "black"),                               # display bezel
        box(0.10, 0.34, 0.02, 0.18, 0.492, 0.50, "black"),       # radio / transponder stack
        box(-0.07, 0.03, -0.06, 0.06, 0.492, 0.50, "black"),     # switch panel
    ]
    for side in (-1, 1):
        cx = side * 0.26
        parts.append(box(cx - 0.19, cx + 0.19, -0.47, -0.39, -0.48, -0.02, "seat"))
        parts.append(box(cx - 0.19, cx + 0.19, -0.44, 0.24, -0.56, -0.47, "seat"))
        parts.append(tube((cx, -0.48, 0.14), (cx, -0.18, 0.19), 0.014, n=8, mat="black"))
        parts.append(ellipsoid((cx, -0.15, 0.19), 0.02, 0.045, 0.02, "black", nu=6, nv=8))
    parts += [
        tube((0.07, -0.04, 0.50), (0.07, -0.045, 0.43), 0.006, n=6, mat="white"),      # throttle rod
        ellipsoid((0.07, -0.045, 0.42), 0.022, 0.022, 0.018, "black", nu=6, nv=10),      # throttle knob
        tube((0.0, -0.485, -0.26), (0.0, -0.30, -0.16), 0.010, n=6, mat="white"),       # flap lever
        ellipsoid((0.0, -0.285, -0.152), 0.018, 0.018, 0.03, "red", nu=6, nv=8),         # flap lever grip
        box(-0.05, 0.05, -0.485, -0.46, -0.40, 0.05, "panel"),                          # centre console
    ]
    return parts


def build_screen():
    screen = Part(closed=False, facing=(0, 0, -1))
    s = SCREEN
    screen.quad((s["x0"], s["y0"], s["z"]), (s["x1"], s["y0"], s["z"]),
                (s["x1"], s["y1"], s["z"]), (s["x0"], s["y1"], s["z"]),
                SCREEN_MATERIAL, uv=((0, 1), (1, 1), (1, 0), (0, 0)))
    return screen


def build_exterior():
    """Return (root nodes, animations) for the airframe."""
    airframe = Node("A240_Exterior")
    airframe.add(build_fuselage(), build_stabilizer(), build_fin(), build_spinner())
    for side in (-1, 1):
        airframe.add(build_wing(side), *build_struts(side), *build_flaperon_brackets(side))
    airframe.add(*build_gear())

    # control surfaces: outer node carries the flap rotation, inner node the aileron rotation
    flap_nodes, ail_nodes = {}, {}
    for side, tag in ((-1, "L"), (1, "R")):
        flap = airframe.child(Node("SKEL_Flap_%s" % tag, FLAPERON_PIVOT))
        ail = flap.child(Node("SKEL_Aileron_%s" % tag, FLAPERON_PIVOT))
        ail.add(build_flaperon(side))
        flap_nodes[tag], ail_nodes[tag] = flap, ail
    elevator = airframe.child(Node("SKEL_Elevator", (0.0, STAB_Y, ELEV_HINGE_Z)))
    elevator.add(build_elevator(-1), build_elevator(1))
    rudder = airframe.child(Node("SKEL_Rudder", (0.0, 0.5, RUDDER_HINGE_Z)))
    rudder.add(build_rudder())
    # The sim spins these two nodes itself (engines.cfg prop_node.0) and the model
    # behaviour swaps the solid blades for the blur disc once the propeller is turning.
    blades = airframe.child(Node("PROPELLER", (0.0, 0.0, PROP_Z)))
    blades.add(*build_blades())
    blur = airframe.child(Node("PROPELLER_Blur", (0.0, 0.0, PROP_Z)))
    blur.add(build_prop_blur())

    # Animations: (frame, degrees) over 100 frames; names are the SDK's standard ones.
    # Axes are in the glTF frame (+X left, +Y up, +Z forward): rotation about +X by a
    # positive angle lifts a trailing edge, about +Y swings it to the right.
    # Frame 100 is +100% deflection, checked against the SDK's DA62 sample.
    X, Y = (1, 0, 0), (0, 1, 0)
    anims = [
        ("elevator_percent_key", [(elevator, X, [(0, -20), (50, 0), (100, 28)])]),
        ("rudder_percent_key", [(rudder, Y, [(0, -25), (50, 0), (100, 25)])]),
        ("l_aileron_percent_key", [(ail_nodes["L"], X, [(0, 20), (50, 0), (100, -15)])]),
        ("r_aileron_percent_key", [(ail_nodes["R"], X, [(0, -15), (50, 0), (100, 20)])]),
        ("l_flap_percent_key", [(flap_nodes["L"], X, [(0, 0), (100, -20)])]),
        ("r_flap_percent_key", [(flap_nodes["R"], X, [(0, 0), (100, -20)])]),
    ]
    return [airframe], anims


def build_interior():
    """Return (root nodes, animations) for the cabin."""
    cabin = Node("A240_Interior")
    cabin.add(*build_cabin())
    cabin.child(Node("EFIS_Screen")).add(build_screen())
    return [cabin], []


FPS = 30.0          # animation keyframes are written at 30 frames per second


# ----------------------------------------------------------------------------
# glTF export
# ----------------------------------------------------------------------------
def srgb_to_linear(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def finalize_part(part, pivot):
    """Convert a Part to glTF-frame, node-local triangles with correct winding and normals.

    Returns a list of (positions[3], normals[3], uvs[3], material).
    """
    tris = []
    for a, b, c, mat, uv in part.tris:
        pts = [(-(p[0] - pivot[0]), p[1] - pivot[1], p[2] - pivot[2]) for p in (a, b, c)]
        if vlen(vcross(vsub(pts[1], pts[0]), vsub(pts[2], pts[0]))) < 1e-10:
            continue                                     # degenerate
        tris.append([pts, list(uv) if uv else [(0.0, 0.0)] * 3, mat])
    if not tris:
        return []

    def flip(t):
        t[0][1], t[0][2] = t[0][2], t[0][1]
        t[1][1], t[1][2] = t[1][2], t[1][1]

    if part.closed:
        # signed volume about the part's own centre (translation invariant for closed meshes)
        vol = sum(vdot(t[0][0], vcross(t[0][1], t[0][2])) for t in tris)
        if (vol < 0) != part.invert:
            for t in tris:
                flip(t)
        tris = [t for t in tris if t[2] != SKIP]
    else:
        f = (-part.facing[0], part.facing[1], part.facing[2])
        for t in tris:
            if vdot(vcross(vsub(t[0][1], t[0][0]), vsub(t[0][2], t[0][0])), f) < 0:
                flip(t)

    # crease-angle smoothing
    cosc = math.cos(math.radians(part.crease))
    fn, fa, adj = [], [], {}
    key = lambda p: (round(p[0], 5), round(p[1], 5), round(p[2], 5))
    for i, t in enumerate(tris):
        n = vcross(vsub(t[0][1], t[0][0]), vsub(t[0][2], t[0][0]))
        fa.append(vlen(n))
        fn.append(vnorm(n))
        for p in t[0]:
            adj.setdefault(key(p), []).append(i)
    out = []
    for i, t in enumerate(tris):
        normals = []
        for p in t[0]:
            s = (0.0, 0.0, 0.0)
            for j in adj[key(p)]:
                if vdot(fn[i], fn[j]) >= cosc:
                    s = vadd(s, vmul(fn[j], fa[j]))
            n = vnorm(s)
            normals.append(n if vlen(n) > 0.5 else fn[i])
        out.append((t[0], normals, t[1], t[2]))
    return out


class GltfWriter:
    def __init__(self, name):
        self.name = name
        self.bin = bytearray()
        self.views, self.accessors = [], []
        self.meshes, self.nodes, self.materials = [], [], []
        self.mat_index = {}
        self.extensions = {"ASOBO_normal_map_convention", "ASOBO_unique_id"}
        self.world_tris = []          # (p0, p1, p2, material) in glTF world frame, for previews
        self.stats = {"triangles": 0, "vertices": 0}

    def _view(self, data, target=None):
        while len(self.bin) % 4:
            self.bin.append(0)
        view = {"buffer": 0, "byteOffset": len(self.bin), "byteLength": len(data)}
        if target:
            view["target"] = target
        self.bin.extend(data)
        self.views.append(view)
        return len(self.views) - 1

    def _accessor(self, data, comp, count, type_, target=None, minmax=None):
        acc = {"bufferView": self._view(data, target), "componentType": comp,
               "count": count, "type": type_}
        if minmax:
            acc["min"], acc["max"] = minmax
        self.accessors.append(acc)
        return len(self.accessors) - 1

    def material(self, name):
        if name in self.mat_index:
            return self.mat_index[name]
        m = MATERIALS[name]
        r, g, b = (round(srgb_to_linear(c), 4) for c in m["rgb"])
        mat = {"name": name,
               "pbrMetallicRoughness": {"baseColorFactor": [r, g, b, m["a"]],
                                        "metallicFactor": m["metal"], "roughnessFactor": m["rough"]}}
        if m["double"]:
            mat["doubleSided"] = True
        if m["a"] < 1.0:
            mat["alphaMode"] = "BLEND"
        if m["emissive"]:
            mat["emissiveFactor"] = [m["emissive"]] * 3
        if m["ext"]:
            mat["extensions"] = m["ext"]
            self.extensions.update(m["ext"].keys())
        self.materials.append(mat)
        self.mat_index[name] = len(self.materials) - 1
        return self.mat_index[name]

    def _primitives(self, node):
        groups = {}
        gp = (-node.pivot[0], node.pivot[1], node.pivot[2])
        for part in node.parts:
            for pts, normals, uvs, mat in finalize_part(part, node.pivot):
                g = groups.setdefault(mat, {"lookup": {}, "pos": [], "nrm": [], "uv": [], "idx": []})
                for p, n, uv in zip(pts, normals, uvs):
                    k = (round(p[0], 5), round(p[1], 5), round(p[2], 5),
                         round(n[0], 3), round(n[1], 3), round(n[2], 3), uv)
                    i = g["lookup"].get(k)
                    if i is None:
                        i = g["lookup"][k] = len(g["pos"])
                        g["pos"].append(p); g["nrm"].append(n); g["uv"].append(uv)
                    g["idx"].append(i)
                self.world_tris.append((vadd(pts[0], gp), vadd(pts[1], gp), vadd(pts[2], gp), mat))
        prims = []
        for mat, g in groups.items():
            count = len(g["pos"])
            assert count < 65536, "primitive too large for 16-bit indices"
            colors = [(1.0, 1.0, 1.0, 1.0)] * count        # the sim expects vertex colours
            lo = [min(p[i] for p in g["pos"]) for i in range(3)]
            hi = [max(p[i] for p in g["pos"]) for i in range(3)]
            pack = lambda rows, n: b"".join(struct.pack("<%df" % n, *r) for r in rows)
            attrs = {
                "POSITION": self._accessor(pack(g["pos"], 3), 5126, count, "VEC3", 34962, (lo, hi)),
                "NORMAL": self._accessor(pack(g["nrm"], 3), 5126, count, "VEC3", 34962),
                "COLOR_0": self._accessor(pack(colors, 4), 5126, count, "VEC4", 34962),
                "TEXCOORD_0": self._accessor(pack(g["uv"], 2), 5126, count, "VEC2", 34962),
            }
            idx = struct.pack("<%dH" % len(g["idx"]), *g["idx"])
            prims.append({"attributes": attrs,
                          "indices": self._accessor(idx, 5123, len(g["idx"]), "SCALAR", 34963),
                          "material": self.material(mat), "mode": 4})
            self.stats["triangles"] += len(g["idx"]) // 3
            self.stats["vertices"] += count
        return prims

    def node(self, node, parent_pivot=(0.0, 0.0, 0.0)):
        index = len(self.nodes)
        self.nodes.append(None)
        entry = {"name": node.name, "extensions": {"ASOBO_unique_id": {"id": node.name}}}
        t = vsub(node.pivot, parent_pivot)
        if vlen(t) > 1e-9:
            entry["translation"] = [round(-t[0], 6), round(t[1], 6), round(t[2], 6)]
        prims = self._primitives(node)
        if prims:
            self.meshes.append({"name": node.name, "primitives": prims})
            entry["mesh"] = len(self.meshes) - 1
        kids = [self.node(c, node.pivot) for c in node.children]
        if kids:
            entry["children"] = kids
        self.nodes[index] = entry
        node.index = index
        return index

    def animations(self, anims):
        out = []
        for name, channels in anims:
            samplers, chans = [], []
            for node, axis, keys in channels:
                times = [f / FPS for f, _ in keys]
                quats = []
                for _, deg in keys:
                    h = math.radians(deg) / 2
                    s = math.sin(h)
                    quats.append((axis[0] * s, axis[1] * s, axis[2] * s, math.cos(h)))
                tin = self._accessor(struct.pack("<%df" % len(times), *times), 5126, len(times),
                                     "SCALAR", None, ([min(times)], [max(times)]))
                tout = self._accessor(b"".join(struct.pack("<4f", *q) for q in quats), 5126,
                                      len(quats), "VEC4")
                samplers.append({"input": tin, "output": tout, "interpolation": "LINEAR"})
                chans.append({"sampler": len(samplers) - 1,
                              "target": {"node": node.index, "path": "rotation"}})
            out.append({"name": name, "samplers": samplers, "channels": chans})
        return out

    def write(self, roots, anims, out_dir):
        root_ids = [self.node(r) for r in roots]
        animations = self.animations(anims)
        doc = {
            "asset": {"version": "2.0", "generator": "a240 build_model.py",
                      "extensions": {"ASOBO_normal_map_convention": {"tangent_space_convention": "DirectX"}}},
            "extensionsUsed": sorted(self.extensions),
            "scene": 0,
            "scenes": [{"nodes": root_ids}],
            "nodes": self.nodes,
            "meshes": self.meshes,
            "materials": self.materials,
            "accessors": self.accessors,
            "bufferViews": self.views,
            "buffers": [{"uri": self.name + ".bin", "byteLength": len(self.bin)}],
        }
        if animations:
            doc["animations"] = animations
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, self.name + ".bin"), "wb") as f:
            f.write(self.bin)
        with open(os.path.join(out_dir, self.name + ".gltf"), "w", encoding="utf-8", newline="\n") as f:
            json.dump(doc, f, separators=(",", ":"))
        return doc


# ----------------------------------------------------------------------------
# preview renderer (orthographic, z-buffered, pure Python)
# ----------------------------------------------------------------------------
def render(world_tris, width, height, view_dir, up_hint=(0, 1, 0), skip=("blur",), strict=False):
    f = vnorm(view_dir)
    r = vnorm(vcross(f, up_hint))
    u = vcross(r, f)
    light = vnorm(vadd(vadd(vmul(f, -0.55), vmul(u, 0.75)), vmul(r, -0.35)))
    proj = []
    for a, b, c, mat in world_tris:
        if mat in skip:
            continue
        n = vnorm(vcross(vsub(b, a), vsub(c, a)))
        proj.append(([(vdot(p, r), vdot(p, u), vdot(p, f)) for p in (a, b, c)], n, mat))
    xs = [p[0] for t in proj for p in t[0]]
    ys = [p[1] for t in proj for p in t[0]]
    margin = 26
    scale = min((width - 2 * margin) / (max(xs) - min(xs)), (height - 2 * margin) / (max(ys) - min(ys)))
    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    bg = (233, 237, 242)
    color = [bg] * (width * height)
    depth = [1e30] * (width * height)

    def draw(pts, rgb, alpha):
        (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = pts
        area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        if abs(area) < 1e-9:
            return
        inv = 1.0 / area
        xa, xb = max(int(min(x0, x1, x2)), 0), min(int(max(x0, x1, x2)) + 1, width - 1)
        ya, yb = max(int(min(y0, y1, y2)), 0), min(int(max(y0, y1, y2)) + 1, height - 1)
        for y in range(ya, yb + 1):
            py = y + 0.5
            row = y * width
            for x in range(xa, xb + 1):
                px = x + 0.5
                w0 = ((x1 - px) * (y2 - py) - (x2 - px) * (y1 - py)) * inv
                if w0 < 0:
                    continue
                w1 = ((x2 - px) * (y0 - py) - (x0 - px) * (y2 - py)) * inv
                if w1 < 0:
                    continue
                w2 = 1.0 - w0 - w1
                if w2 < 0:
                    continue
                z = w0 * z0 + w1 * z1 + w2 * z2
                i = row + x
                if z < depth[i]:
                    if alpha >= 1.0:
                        depth[i] = z
                        color[i] = rgb
                    else:
                        o = color[i]
                        color[i] = (int(o[0] + (rgb[0] - o[0]) * alpha),
                                    int(o[1] + (rgb[1] - o[1]) * alpha),
                                    int(o[2] + (rgb[2] - o[2]) * alpha))

    for transparent in (False, True):
        for pts, n, mat in proj:
            m = MATERIALS[mat]
            (cr, cg, cb), alpha, double = m["rgb"], m["a"], m["double"]
            if (alpha < 1.0) != transparent:
                continue
            facing = vdot(n, f) < 0
            if not facing and (strict and alpha >= 1.0 or not double):
                continue                              # back-face culling exposes winding errors
            nn = n if facing else vmul(n, -1)
            shade = 0.34 + 0.66 * max(0.0, vdot(nn, light))
            rgb = (int(min(255, cr * 255 * shade)), int(min(255, cg * 255 * shade)),
                   int(min(255, cb * 255 * shade)))
            screen = [((p[0] - cx) * scale + width / 2, height / 2 - (p[1] - cy) * scale, p[2])
                      for p in pts]
            draw(screen, rgb, alpha if transparent else 1.0)
    return color


def write_png(path, width, height, pixels):
    raw = bytearray()
    for y in range(height):
        raw.append(0)
        for r, g, b in pixels[y * width:(y + 1) * width]:
            raw.extend((r, g, b))
    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(bytes(raw), 6)) + chunk(b"IEND", b""))


# view directions are in the glTF frame: +X = left wing, +Y = up, +Z = nose
VIEWS = [
    ("three_quarter", (0.62, -0.38, -0.69), (0, 1, 0)),   # from front-right, above
    ("side", (1, 0, 0), (0, 1, 0)),                       # from the right side... see note below
    ("top", (0, -1, 0), (0, 0, 1)),
    ("front", (0, 0, -1), (0, 1, 0)),
]


def render_previews(world_tris, tag, w=820, h=470, strict=False):
    os.makedirs(PREVIEW_DIR, exist_ok=True)
    sheet = [(233, 237, 242)] * (2 * w * 2 * h)
    for i, (name, direction, up) in enumerate(VIEWS):
        img = render(world_tris, w, h, direction, up, strict=strict)
        write_png(os.path.join(PREVIEW_DIR, "%s_%s.png" % (tag, name)), w, h, img)
        ox, oy = (i % 2) * w, (i // 2) * h
        for y in range(h):
            row = (oy + y) * 2 * w + ox
            sheet[row:row + w] = img[y * w:(y + 1) * w]
    path = os.path.join(PREVIEW_DIR, "%s_sheet.png" % tag)
    write_png(path, 2 * w, 2 * h, sheet)
    return path


# file name, width, height, view direction (glTF frame), up
THUMBNAILS = [
    ("thumbnail.png", 720, 344, (0.62, -0.30, -0.72), (0, 1, 0)),
    ("thumbnail_button.png", 830, 260, (0.62, -0.30, -0.72), (0, 1, 0)),
    ("thumbnail_variation.png", 660, 300, (0.62, -0.30, -0.72), (0, 1, 0)),
    ("thumbnail_side.png", 930, 340, (1, 0, 0), (0, 1, 0)),
]


def main():
    previews = "--no-preview" not in sys.argv
    strict = "--strict" in sys.argv
    world = []
    for name, build in (("A240", build_exterior), ("A240_interior", build_interior)):
        roots, anims = build()
        writer = GltfWriter(name)
        writer.write(roots, anims, MODEL_DIR)
        world += writer.world_tris
        print("%s.gltf: %d triangles, %d vertices, %d nodes, %d animations, %d KB"
              % (name, writer.stats["triangles"], writer.stats["vertices"], len(writer.nodes),
                 len(anims), len(writer.bin) // 1024))
    xs = [p[0] for t in world for p in t[:3]]
    ys = [p[1] for t in world for p in t[:3]]
    zs = [p[2] for t in world for p in t[:3]]
    print("span %.2f m, length %.2f m, height %.2f m (ground at y=%.2f, lowest point %.2f)"
          % (max(xs) - min(xs), max(zs) - min(zs), max(ys) - GROUND_Y, GROUND_Y, min(ys)))
    if previews:
        print("preview:", render_previews(world, "A240" + ("_strict" if strict else ""), strict=strict))
        if not strict:
            os.makedirs(THUMBNAIL_DIR, exist_ok=True)
            for fname, w, h, direction, up in THUMBNAILS:
                write_png(os.path.join(THUMBNAIL_DIR, fname), w, h, render(world, w, h, direction, up))
            print("thumbnails:", THUMBNAIL_DIR)


if __name__ == "__main__":
    main()
