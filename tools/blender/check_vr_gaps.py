"""Check the model for the gaps you would notice in VR.

    blender -b blender/A240.blend --python-exit-code 1 --python tools/blender/check_vr_gaps.py [-- --rays N] [--frames 0,50,100]

In VR you lean into every corner of the cockpit, so seams a fixed 2D camera never shows
become obvious. The sim draws both models in the cockpit (model.cfg
withInterior_showExterior) and does not draw the back faces of single-sided materials, so a
crack between two parts shows whatever lies behind - often daylight. Three checks:

1. See-through: from ~370 head positions around both seats (leaning to the panel, the
   windows, the floor, behind the seats), rays in every direction, traced with the sim's
   rules. A ray whose first opaque hit is a back face, or that leaves the aircraft
   without passing through glass, is a gap.
2. Floating parts: mesh islands that neither touch nor cut into any other part.
3. Poke-through: cabin parts that stick out through the painted skin (window reveals
   may, inside the rubber seals).

All three run with the animations at keys 0, 50 and 100 - controls at both ends of their
travel and centred - so a moving part cannot open a gap at full deflection either.
Exits with code 1 if anything is found.
"""
import math
import os
import sys
from collections import defaultdict

import bmesh
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a240_lib import MATERIALS  # noqa: E402

ARGS = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
NRAYS = int(ARGS[ARGS.index("--rays") + 1]) if "--rays" in ARGS else 2000
FRAMES = [int(f) for f in ARGS[ARGS.index("--frames") + 1].split(",")] if "--frames" in ARGS else [0, 50, 100]
TRANSPARENT = {"glass", "glass_tint", "blur"}
DOUBLE_SIDED = {k for k, v in MATERIALS.items() if v["extra"].get("msfs_double_sided")}
SKIP = {"PROPELLER_Blur"}                # only shown while the propeller turns
MIN_GAP = 0.0005                         # parts further apart than this are not touching


def B(p):
    return Vector((-p[0], -p[2], p[1]))


def S(p):
    return tuple(round(c, 3) for c in (-p[0], p[2], -p[1]))


def load(frame):
    """All triangles in world space at an animation key, with object, material, island, normal."""
    bpy.context.scene.frame_set(frame)
    dg = bpy.context.evaluated_depsgraph_get()
    m = dict(verts=[], tris=[], obj=[], mat=[], isl=[], nrm=[], islands=[])
    for o in bpy.data.objects:
        if o.type != "MESH" or o.name in SKIP:
            continue
        bm = bmesh.new()
        bm.from_mesh(o.evaluated_get(dg).data)
        bm.transform(o.matrix_world)
        mats = [s.material.name.split(".")[0] if s.material else "?" for s in o.material_slots]
        bmesh.ops.triangulate(bm, faces=bm.faces[:])
        bm.verts.index_update()
        bm.faces.index_update()
        faces = bm.faces
        off = len(m["verts"])
        m["verts"].extend(v.co.copy() for v in bm.verts)
        seen = {}
        for f in faces:                                # islands by shared vertices
            if f.index in seen:
                continue
            iid = len(m["islands"])
            m["islands"].append(o.name)
            stack = [f]
            seen[f.index] = iid
            while stack:
                g = stack.pop()
                for v in g.verts:
                    for h in v.link_faces:
                        if h.index not in seen:
                            seen[h.index] = iid
                            stack.append(h)
        for f in faces:
            a, b, c = (off + v.index for v in f.verts)
            n = (m["verts"][b] - m["verts"][a]).cross(m["verts"][c] - m["verts"][a])
            if n.length < 1e-14:
                continue
            m["tris"].append((a, b, c))
            m["obj"].append(o.name)
            m["mat"].append(mats[f.material_index] if mats else "?")
            m["isl"].append(seen[f.index])
            m["nrm"].append(n.normalized())
        bm.free()
    m["bvh"] = BVHTree.FromPolygons(m["verts"], m["tris"], all_triangles=True)
    return m


def fib_dirs(n):
    ga = math.pi * (3 - math.sqrt(5))
    out = []
    for i in range(n):
        y = 1 - 2 * (i + 0.5) / n
        r = math.sqrt(1 - y * y)
        out.append(Vector((r * math.cos(ga * i), r * math.sin(ga * i), y)))
    return out


def head_positions():
    eyes = []
    for seat in (-1, 1):
        for x in (0.0, 0.08, 0.18, 0.28, 0.36, 0.44):
            for y in (0.02, 0.14, 0.26, 0.34, 0.44):
                for z in (-0.32, -0.16, 0.0, 0.14, 0.28, 0.36):
                    eyes.append((seat * x, y, z))
    for x in (-0.15, 0.0, 0.15):                       # behind the seats
        for z in (-0.55, -0.70):
            eyes.append((x, 0.30, z))
    return eyes


def see_through(m):
    bvh, dirs = m["bvh"], fib_dirs(NRAYS)
    found = defaultdict(lambda: [0, None])
    used = 0
    for e in head_positions():
        o = B(e)
        if bvh.find_nearest(o)[3] < 0.035:            # head inside or touching a part
            continue
        used += 1
        for d in dirs:
            p, glass = o.copy(), False
            for _ in range(64):
                loc, _, i, _ = bvh.ray_cast(p, d)
                if loc is None:
                    if not glass:
                        found[("(nothing - open to the outside)", "")][0] += 1
                    break
                if m["mat"][i] in TRANSPARENT:
                    glass, p = True, loc + d * 1e-5
                    continue
                if m["mat"][i] not in DOUBLE_SIDED and m["nrm"][i].dot(d) > 0:
                    rec = found[(m["obj"][i], m["mat"][i])]
                    rec[0] += 1
                    rec[1] = rec[1] or (S(loc), e)
                break
    print("see-through: %d head positions x %d rays" % (used, NRAYS))
    for (obj, mat), (n, ex) in sorted(found.items(), key=lambda kv: -kv[1][0]):
        print("  GAP %6d rays: back of %s %s, e.g. at %s seen from %s" % (n, obj, mat, ex and ex[0], ex and ex[1]))
    return sum(n for n, _ in found.values())


def floating(m):
    bvh, verts, tris, isl = m["bvh"], m["verts"], m["tris"], m["isl"]
    by_isl = defaultdict(list)
    for t, i in enumerate(isl):
        by_isl[i].append(t)
    touch = defaultdict(set)
    for i, tl in by_isl.items():
        sub = BVHTree.FromPolygons(verts, [tris[t] for t in tl], all_triangles=True)
        touch[i].update(isl[j] for _, j in sub.overlap(bvh) if isl[j] != i)
        for v in {v for t in tl for v in tris[t]}:
            touch[i].update(isl[j] for _, _, j, _ in bvh.find_nearest_range(verts[v], MIN_GAP) if isl[j] != i)
    for i in list(touch):
        for j in touch[i]:
            touch[j].add(i)
    # everything must hang together with the skin or the cabin lining
    reached = {i for i, name in enumerate(m["islands"]) if name in ("Fuselage", "Cabin_Liner")}
    stack = list(reached)
    while stack:
        for j in touch[stack.pop()]:
            if j not in reached:
                reached.add(j)
                stack.append(j)
    bad = [i for i in by_isl if i not in reached]
    print("floating parts: %d of %d islands" % (len(bad), len(by_isl)))
    for i in bad:
        vs = [verts[v] for t in by_isl[i] for v in tris[t]]
        c = sum(vs, Vector()) / len(vs)
        print("  FLOATING %s %s near %s" % (m["islands"][i], m["mat"][by_isl[i][0]], S(c)))
    return len(bad)


def poke_through(m):
    interior = {o.name for o in bpy.data.collections["Interior"].all_objects}
    verts, tris = m["verts"], m["tris"]

    def bvh_of(pred):
        ts = [t for t in range(len(tris)) if pred(t)]
        return BVHTree.FromPolygons(verts, [tris[t] for t in ts], all_triangles=True), ts
    skin, skin_t = bvh_of(lambda t: m["obj"][t] == "Fuselage")
    seal, seal_t = bvh_of(lambda t: m["mat"][t] == "rubber" and m["obj"][t] == "Fuselage_Details")
    owner = {v: m["obj"][t] for t in range(len(tris)) if m["obj"][t] in interior for v in tris[t]}
    bad = defaultdict(lambda: [0, 0.0, None])
    for v in owner:
        p = verts[v]
        loc, _, k, dist = skin.find_nearest(p)
        t = skin_t[k]
        if dist < 0.0002 or m["mat"][t] in TRANSPARENT or (p - loc).dot(m["nrm"][t]) <= 0:
            continue
        sl, _, sk, _ = seal.find_nearest(p)
        if sl is not None and (p - sl).dot(m["nrm"][seal_t[sk]]) < 0:
            continue                                   # hidden inside a window seal
        rec = bad[owner[v]]
        rec[0] += 1
        if dist > rec[1]:
            rec[1], rec[2] = dist, S(p)
    print("poke-through: %d cabin parts outside the painted skin" % len(bad))
    for obj, (n, d, p) in bad.items():
        print("  POKES %s: %d vertices, up to %.1f mm, e.g. at %s" % (obj, n, d * 1000, p))
    return len(bad)


def main():
    problems = 0
    for frame in FRAMES:
        m = load(frame)
        print("== animation key %d: %d triangles; double-sided materials: %s"
              % (frame, len(m["tris"]), ", ".join(sorted(DOUBLE_SIDED))))
        problems += see_through(m) + floating(m) + poke_through(m)
    if problems:
        raise SystemExit("VR gap check failed")
    print("VR gap check passed")


if __name__ == "__main__":
    main()
