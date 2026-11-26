"""Shared helpers for the A240 Blender scripts.

Geometry is written in the project's "sim frame" (metres): +x right wing, +y up, +z nose,
origin on the centreline at the wing quarter-chord and propeller height - the same datum
as flight_model.cfg. Blender is Z-up with the nose towards -Y, so every point goes
through B(): sim (x, y, z) -> Blender (-x, -z, y). The glTF exporter then turns that
back into glTF (+X left wing, +Y up, +Z nose), which is what the sim expects.
"""
import math
import os
import sys

import bmesh
import bpy
from mathutils import Matrix, Quaternion, Vector

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BLEND_PATH = os.path.join(ROOT, "blender", "A240.blend")
MODEL_DIR = os.path.join(ROOT, "sdk-project", "PackageSources", "SimObjects", "Airplanes",
                         "Local_A240", "common", "model")
THUMBNAIL_DIR = os.path.join(ROOT, "sdk-project", "PackageSources", "SimObjects", "Airplanes",
                             "Local_A240", "presets", "local", "A240", "thumbnail")
PREVIEW_DIR = os.path.join(ROOT, "previews")

SDK_BLENDER = os.path.join(os.environ.get("MSFS2024_SDK", r"C:\MSFS 2024 SDK"), "Tools", "Blender")
SCREEN_MATERIAL = "EFIS_Display"     # must match "texture=" in common/panel/panel.cfg
FPS = 30                             # animation keys are frames 0..100 at 30 fps


def enable_msfs_exporter():
    """Load the MSFS 2024 glTF exporter that ships with the SDK (no Blender setup needed)."""
    import addon_utils
    for sub in ("addons", "modules"):
        p = os.path.join(SDK_BLENDER, sub)
        if p not in sys.path:
            sys.path.append(p)
    # default_set=True registers it in the session preferences, which is how the
    # Khronos exporter finds the MSFS export hooks. Nothing is saved to disk.
    addon_utils.enable("io_scene_gltf2_msfs_2024", default_set=True, persistent=False)
    if not hasattr(bpy.types.Material, "msfs_material_type"):
        raise RuntimeError("MSFS 2024 exporter not found under %s" % SDK_BLENDER)


# ----------------------------------------------------------------------------
# frames and small maths
# ----------------------------------------------------------------------------
def V(*a):
    return Vector(a[0]) if len(a) == 1 else Vector(a)


def B(p):
    return Vector((-p[0], -p[2], p[1]))


def centroid(pts):
    c = Vector((0.0, 0.0, 0.0))
    for p in pts:
        c += Vector(p)
    return c / len(pts)


def srgb_to_linear(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def rot_x(deg):
    return Matrix.Rotation(math.radians(deg), 3, "X")


def rot_y(deg):
    return Matrix.Rotation(math.radians(deg), 3, "Y")


def rot_z(deg):
    return Matrix.Rotation(math.radians(deg), 3, "Z")


# ----------------------------------------------------------------------------
# materials (MSFS 2024 exporter properties)
# ----------------------------------------------------------------------------
def _m(rgb, a=1.0, metal=0.0, rough=0.5, kind="msfs_standard", **extra):
    return dict(rgb=rgb, a=a, metal=metal, rough=rough, kind=kind, extra=extra)


GLASS = dict(msfs_alpha_mode="BLEND", msfs_double_sided=True, msfs_glass_width=0.5,
             msfs_draw_order_offset=2)
MATERIALS = {
    # paint
    "paint_white":  _m((0.93, 0.93, 0.91), rough=0.32),
    "paint_red":    _m((0.74, 0.06, 0.07), rough=0.30),
    # glazing: the sim lights blended glass in proportion to alpha, so keep it low
    # (high alpha turns milky white in sun). Values follow the SDK's DA62 sample.
    "glass":        _m((0.86, 0.86, 0.85), a=0.03, rough=0.05, kind="msfs_glass", **GLASS),
    "glass_tint":   _m((0.20, 0.27, 0.24), a=0.10, rough=0.05, kind="msfs_glass", **GLASS),
    # trim and hardware
    # double-sided: the window seals cross the glass, and at their joints a stray back
    # face would otherwise show a pinhole of sky
    "rubber":       _m((0.035, 0.035, 0.035), rough=0.80, msfs_double_sided=True),
    "tire":         _m((0.03, 0.03, 0.03), rough=0.92),
    "black":        _m((0.04, 0.04, 0.04), rough=0.55),
    "inlet_dark":   _m((0.012, 0.012, 0.012), rough=0.95),
    "metal":        _m((0.80, 0.80, 0.82), metal=1.0, rough=0.35),
    "metal_dark":   _m((0.33, 0.30, 0.28), metal=1.0, rough=0.55),
    "chrome":       _m((0.92, 0.92, 0.92), metal=1.0, rough=0.12),
    "prop":         _m((0.90, 0.90, 0.88), rough=0.35),     # white composite blades
    "prop_tip":     _m((0.74, 0.06, 0.07), rough=0.35),     # red tips
    "lens_red":     _m((0.85, 0.03, 0.03), rough=0.08),
    "lens_green":   _m((0.05, 0.75, 0.15), rough=0.08),
    "lens_clear":   _m((0.86, 0.88, 0.90), metal=0.6, rough=0.06),
    "blur":         _m((0.10, 0.10, 0.10), a=0.16, rough=0.50, msfs_alpha_mode="BLEND",
                       msfs_double_sided=True, msfs_draw_order_offset=3),
    # cabin
    "panel":        _m((0.17, 0.18, 0.19), rough=0.70),
    "liner":        _m((0.62, 0.63, 0.64), rough=0.85),
    "carpet":       _m((0.16, 0.16, 0.17), rough=0.97),
    "frame":        _m((0.52, 0.53, 0.55), metal=0.3, rough=0.55),
    "seat":         _m((0.30, 0.10, 0.09), rough=0.60),
    "seat_fabric":  _m((0.11, 0.11, 0.12), rough=0.92),
    "belt":         _m((0.07, 0.10, 0.24), rough=0.90),
    "gauge_face":   _m((0.02, 0.02, 0.02), rough=0.45),
    "gauge_mark":   _m((0.92, 0.92, 0.90), rough=0.50),
    "arc_green":    _m((0.08, 0.60, 0.15), rough=0.50),
    "arc_yellow":   _m((0.90, 0.75, 0.05), rough=0.50),
    "arc_red":      _m((0.80, 0.05, 0.05), rough=0.50),
    "arc_white":    _m((0.85, 0.85, 0.85), rough=0.50),
    "lcd":          _m((0.06, 0.09, 0.07), rough=0.15),
    "knob_red":     _m((0.74, 0.06, 0.07), rough=0.45),
    "knob_yellow":  _m((0.98, 0.68, 0.04), rough=0.40),
    "placard":      _m((0.95, 0.80, 0.10), rough=0.60),
    # instrument panel (colours sampled from Aeropro's render of the Garmin panel)
    "panel_grey":   _m((0.40, 0.42, 0.44), rough=0.55),
    "label":        _m((0.03, 0.03, 0.03), rough=0.40),
    "label_text":   _m((0.80, 0.80, 0.80), rough=0.60),
    "bezel":        _m((0.07, 0.07, 0.075), rough=0.35),
    "button":       _m((0.16, 0.16, 0.17), rough=0.45),
    "lcd_light":    _m((0.62, 0.65, 0.62), rough=0.25),
    # the cockpit display: the sim paints the HTML instrument onto this material
    SCREEN_MATERIAL: _m((0.32, 0.32, 0.32), rough=0.80, msfs_emissive_factor=(1.0, 1.0, 1.0),
                        msfs_emissive_scale=5000.0),
}


def get_material(name):
    mat = bpy.data.materials.get(name)
    if mat:
        return mat
    spec = MATERIALS[name]
    mat = bpy.data.materials.new(name)
    mat.msfs_material_type = spec["kind"]           # builds the exporter's node tree
    rgb = [srgb_to_linear(c) for c in spec["rgb"]]
    mat.msfs_base_color_factor = (rgb[0], rgb[1], rgb[2], spec["a"])
    mat.msfs_metallic_factor = spec["metal"]
    mat.msfs_roughness_factor = spec["rough"]
    for k, v in spec["extra"].items():
        setattr(mat, k, v)
    mat.diffuse_color = (rgb[0], rgb[1], rgb[2], spec["a"])   # viewport solid colour
    return mat


# ----------------------------------------------------------------------------
# mesh building
# ----------------------------------------------------------------------------
class MB:
    """Collects vertices (sim frame) and polygons (with a material name each)."""

    def __init__(self):
        self.verts = []
        self.faces = []
        self.fmat = []
        self.uv = {}            # face index -> list of (u, v) per corner (optional)

    def v(self, p):
        self.verts.append(Vector(p))
        return len(self.verts) - 1

    def f(self, idx, mat, uv=None):
        self.faces.append(list(idx))
        self.fmat.append(mat)
        if uv:
            self.uv[len(self.faces) - 1] = uv

    def f_toward(self, idx, mat, direction):
        """Add a face wound so that its front side looks along `direction` (sim frame).

        The sim culls back faces, so for single-sided sheets the winding decides which side
        can be seen. B() mirrors (det -1), so the winding is checked in Blender space."""
        pts = [B(self.verts[i]) for i in idx]
        n = Vector((0.0, 0.0, 0.0))
        for a, b in zip(pts, pts[1:] + pts[:1]):           # Newell normal
            n += a.cross(b)
        if n.dot(B(direction)) < 0:
            idx = list(idx)[::-1]
        self.f(idx, mat)

    def extend(self, other):
        off = len(self.verts)
        self.verts.extend(other.verts)
        for i, (face, mat) in enumerate(zip(other.faces, other.fmat)):
            self.faces.append([j + off for j in face])
            self.fmat.append(mat)
            if i in other.uv:
                self.uv[len(self.faces) - 1] = other.uv[i]
        return self

    # -- primitives ------------------------------------------------------------
    def loft(self, rings, mat, cap0=True, cap1=True, closed=True):
        """Skin rings (lists of equal length) with quads; material may be a function of the face centre."""
        pick = mat if callable(mat) else (lambda c: mat)
        idx = [[self.v(p) for p in ring] for ring in rings]
        n = len(rings[0])
        m = n if closed else n - 1
        for a, b in zip(idx, idx[1:]):
            for j in range(m):
                k = (j + 1) % n
                q = [a[j], a[k], b[k], b[j]]
                self.f(q, pick(centroid([self.verts[i] for i in q])))
        if cap0:
            self.f(idx[0][::-1], pick(centroid([self.verts[i] for i in idx[0]])))
        if cap1:
            self.f(idx[-1], pick(centroid([self.verts[i] for i in idx[-1]])))
        return idx

    def sweep(self, pts, radius=0.01, n=8, mat="black", cap=True, closed=False, profile=None,
              ups=None, up=None, radii=None, flat=1.0):
        """Sweep a circle (or a 2D profile in the (u, v) frame) along a polyline.

        u follows `up` (or `ups` per point) projected off the tangent, else a parallel-transported
        frame. A profile is a list of (a, b) offsets along (u, v).
        """
        pts = [Vector(p) for p in pts]
        cnt = len(pts)
        tans = []
        for i in range(cnt):
            if closed:
                t = pts[(i + 1) % cnt] - pts[i - 1]
            else:
                t = pts[min(i + 1, cnt - 1)] - pts[max(i - 1, 0)]
            tans.append(t.normalized())
        frames = []
        u = None
        for i, t in enumerate(tans):
            want = ups[i] if ups else up
            if want is not None:
                w = Vector(want)
                u = (w - t * w.dot(t)).normalized()
            elif u is None:
                ref = Vector((0, 0, 1)) if abs(t.z) < 0.9 else Vector((1, 0, 0))
                u = t.cross(ref).normalized()
            else:
                u = (u - t * u.dot(t)).normalized()
            frames.append((u, t.cross(u)))
        if profile is None:
            profile = [(math.cos(2 * math.pi * j / n), flat * math.sin(2 * math.pi * j / n)) for j in range(n)]
            scale = True
        else:
            scale = False
        rings = []
        for i, (p, (u, v)) in enumerate(zip(pts, frames)):
            r = radii[i] if radii else radius
            s = r if scale else 1.0
            rings.append([p + u * (a * s) + v * (b * s) for a, b in profile])
        if closed:
            rings.append(rings[0])
            idx = [[self.v(p) for p in ring] for ring in rings[:-1]]
            idx.append(idx[0])
            m = len(profile)
            pick = mat if callable(mat) else (lambda c: mat)
            for a, b in zip(idx, idx[1:]):
                for j in range(m):
                    k = (j + 1) % m
                    q = [a[j], a[k], b[k], b[j]]
                    self.f(q, pick(centroid([self.verts[x] for x in q])))
            return
        self.loft(rings, mat, cap0=cap, cap1=cap)

    def tube(self, p0, p1, r0, r1=None, n=12, mat="black", flat=1.0, up=None):
        r1 = r0 if r1 is None else r1
        self.sweep([p0, p1], n=n, mat=mat, radii=[r0, r1], flat=flat, up=up)

    def disc(self, c, normal, r, thick, n=16, mat="black", up=None):
        c, nrm = Vector(c), Vector(normal).normalized()
        self.sweep([c - nrm * (thick / 2), c + nrm * (thick / 2)], radius=r, n=n, mat=mat, up=up)

    def ellipsoid(self, c, rx, ry, rz, mat, nu=10, nv=16, rot=None):
        c = Vector(c)
        rings = []
        for i in range(nu + 1):
            th = math.radians(4 + 172.0 * i / nu)
            ring = []
            for j in range(nv):
                ph = 2 * math.pi * j / nv
                p = Vector((rx * math.sin(th) * math.cos(ph), ry * math.sin(th) * math.sin(ph), rz * math.cos(th)))
                ring.append(c + (rot @ p if rot else p))
            rings.append(ring)
        self.loft(rings, mat)

    def box(self, x0, x1, y0, y1, z0, z1, mat):
        r0 = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)]
        r1 = [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        self.loft([r0, r1], mat)

    def rbox(self, center, size, radius, mat, segs=2, rot=None):
        """Box with bevelled edges. size = full extents (x, y, z); rot = 3x3 matrix."""
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1.0)
        for vert in bm.verts:
            vert.co = Vector((vert.co.x * size[0], vert.co.y * size[1], vert.co.z * size[2]))
        r = min(radius, min(size) * 0.49)
        if r > 1e-5:
            bmesh.ops.bevel(bm, geom=list(bm.edges) + list(bm.verts), offset=r, segments=segs,
                            affect="EDGES", profile=0.5)
        c = Vector(center)
        off = len(self.verts)
        for vert in bm.verts:
            p = rot @ vert.co if rot else vert.co
            self.verts.append(c + p)
        for face in bm.faces:
            self.faces.append([off + vv.index for vv in face.verts])
            self.fmat.append(mat)
        bm.free()


def _box_uvs(me):
    """Placeholder UVs (box projection, 1 unit = 1 m) so every mesh has a UV map."""
    uv = me.uv_layers.new(name="UVMap")
    for poly in me.polygons:
        n = poly.normal
        ax = max(range(3), key=lambda i: abs(n[i]))
        a, b = [(1, 2), (0, 2), (0, 1)][ax]
        for li in poly.loop_indices:
            co = me.vertices[me.loops[li].vertex_index].co
            uv.data[li].uv = (co[a], co[b])
    return uv


def make_object(name, mb, collection, parent=None, pivot=(0, 0, 0), smooth=35.0,
                normals="outside", weld=1e-5):
    """Turn an MB into a mesh object whose origin sits at `pivot` (sim frame).

    normals: "outside" recalculates per closed island; a function f(sim_point) -> sim vector
    orients each face towards that direction (for open sheets); None keeps the winding as
    built (see MB.f_toward).
    """
    pivot = Vector(pivot)
    me = bpy.data.meshes.new(name)
    me.from_pydata([B(p - pivot) for p in mb.verts], [], mb.faces)
    names = []
    for m in mb.fmat:
        if m not in names:
            names.append(m)
    for m in names:
        me.materials.append(get_material(m))
    me.polygons.foreach_set("material_index", [names.index(m) for m in mb.fmat])
    custom_uv = bool(mb.uv)
    if custom_uv:
        uv = me.uv_layers.new(name="UVMap")
        for i, poly in enumerate(me.polygons):
            if i in mb.uv:
                for li, coord in zip(poly.loop_indices, mb.uv[i]):
                    uv.data[li].uv = coord
    me.update()

    bm = bmesh.new()
    bm.from_mesh(me)
    if weld:
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=weld)
    bmesh.ops.dissolve_degenerate(bm, edges=bm.edges, dist=1e-7)
    if normals == "outside":
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    elif callable(normals):
        for face in bm.faces:
            c = face.calc_center_median() + B(pivot)
            sim_c = Vector((-c.x, c.z, -c.y))
            if face.normal.dot(B(normals(sim_c))) < 0:
                face.normal_flip()
    bm.to_mesh(me)
    bm.free()
    if not custom_uv:
        _box_uvs(me)
    # The MSFS shader multiplies the base colour by this white vertex-colour layer; a mesh
    # without it renders black in Blender (the exporter drops an all-white layer anyway).
    col = me.color_attributes.new("Color", "BYTE_COLOR", "CORNER")
    col.data.foreach_set("color", [1.0] * (4 * len(me.loops)))
    me.color_attributes.active_color = col
    me.color_attributes.render_color_index = me.color_attributes.find("Color")
    finish_shading(me, smooth)

    obj = bpy.data.objects.new(name, me)
    collection.objects.link(obj)
    place(obj, pivot, parent)
    return obj


def finish_shading(me, smooth):
    me.shade_smooth()
    if smooth is not None:
        me.set_sharp_from_angle(angle=math.radians(smooth))


def place(obj, pivot, parent=None):
    """Position obj at sim-frame pivot, under parent (whose pivot is in obj['pivot'] terms)."""
    obj["sim_pivot"] = list(pivot)
    if parent is not None:
        obj.parent = parent
        obj.matrix_parent_inverse = Matrix.Identity(4)
        obj.location = B(Vector(pivot) - Vector(parent.get("sim_pivot", (0, 0, 0))))
    else:
        obj.location = B(pivot)


def make_empty(name, collection, parent=None, pivot=(0, 0, 0), size=0.15):
    obj = bpy.data.objects.new(name, None)
    obj.empty_display_type = "PLAIN_AXES"
    obj.empty_display_size = size
    collection.objects.link(obj)
    place(obj, pivot, parent)
    return obj


def boolean_apply(obj, cutters, operation="DIFFERENCE"):
    """Apply exact booleans with each cutter object, then delete the cutters."""
    for cutter in cutters:
        mod = obj.modifiers.new("cut", "BOOLEAN")
        mod.operation = operation
        mod.solver = "EXACT"
        mod.object = cutter
        if hasattr(mod, "material_mode"):
            mod.material_mode = "TRANSFER"
    dg = bpy.context.evaluated_depsgraph_get()
    new = bpy.data.meshes.new_from_object(obj.evaluated_get(dg))
    old = obj.data
    obj.modifiers.clear()
    obj.data = new
    new.name = old.name
    bpy.data.meshes.remove(old)
    for cutter in cutters:
        me = cutter.data
        bpy.data.objects.remove(cutter)
        bpy.data.meshes.remove(me)
    return obj


def cutter_object(mb, collection):
    """A throwaway object for boolean cuts (positioned in world space)."""
    return make_object("_cutter", mb, collection, smooth=None)


# ----------------------------------------------------------------------------
# section shapes
# ----------------------------------------------------------------------------
def airfoil_loop(m, p, t, x0=0.0, x1=1.0, n=12):
    """NACA 4-digit section between chord fractions x0..x1 as a closed loop of (xc, yc),
    upper surface from trailing edge to leading edge, then lower surface back."""
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
        lo = lo[1:]
    if x1 >= 1 - 1e-9:
        lo = lo[:-1]
    return up[::-1] + lo


def airfoil_surface(m, p, t, x, upper=True):
    """(yc, unit normal 2D) on a NACA 4-digit section at chord fraction x."""
    loop_pts = airfoil_loop(m, p, t, max(0.0, x - 0.002), min(1.0, x + 0.002), n=1)
    # loop is [up(x1), up(x0), lo(x0), lo(x1)] (no shared points away from the ends)
    a, b = (loop_pts[1], loop_pts[0]) if upper else (loop_pts[2], loop_pts[3])
    dx, dy = b[0] - a[0], b[1] - a[1]
    ln = math.hypot(dx, dy)
    nx, ny = (-dy / ln, dx / ln) if upper else (dy / ln, -dx / ln)
    return (a[1] + b[1]) / 2, (nx, ny)


def section(origin, chord, chord_dir, thick_dir, loop):
    o, cd, td = Vector(origin), Vector(chord_dir), Vector(thick_dir)
    return [o + cd * (xc * chord) + td * (yc * chord) for xc, yc in loop]
