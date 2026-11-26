"""Export blender/A240.blend to the SDK model folder, and optionally render previews.

    blender -b blender/A240.blend --python tools/blender/export_a240.py -- [options]

Options (after the "--"):
    --no-export     skip the glTF export
    --previews      render previews/A240_*.png and the preset thumbnails
    --cockpit       also render the cockpit views (previews/cockpit_*.png)
    --samples N     render samples (default 32)

The export uses the MSFS 2024 exporter that ships with the SDK, driven by the two
presets stored in the .blend ("A240" = Exterior collection, "A240_interior" = Interior
collection). It is loaded from the SDK folder for this run only - your Blender
preferences are not changed.
"""
import json
import math
import os
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a240_lib import B, MODEL_DIR, PREVIEW_DIR, THUMBNAIL_DIR, enable_msfs_exporter  # noqa: E402

ARGS = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
EXPECTED_ANIMS = {
    "elevator_percent_key": "SKEL_Elevator", "rudder_percent_key": "SKEL_Rudder",
    "l_aileron_percent_key": "SKEL_Aileron_L", "r_aileron_percent_key": "SKEL_Aileron_R",
    "l_flap_percent_key": "SKEL_Flap_L", "r_flap_percent_key": "SKEL_Flap_R",
}
# cockpit controls, driven by model/A240_Interior_Behavior.xml
EXPECTED_INTERIOR_ANIMS = {"throttle_knob": "THROTTLE_Knob", "flap_lever": "HANDLING_Flap_Lever",
                           "elevator_trim_lever": "HANDLING_Trim_Lever"}
for _seat in ("Pilot", "Copilot"):
    EXPECTED_INTERIOR_ANIMS["stick_roll_" + _seat.lower()] = "HANDLING_Stick_Roll_" + _seat
    EXPECTED_INTERIOR_ANIMS["stick_pitch_" + _seat.lower()] = "HANDLING_Stick_Pitch_" + _seat
    for _side in ("L", "R"):
        EXPECTED_INTERIOR_ANIMS["rudder_pedal_%s_%s" % (_side.lower(), _seat.lower())] = \
            "HANDLING_Pedal_%s_%s" % (_side, _seat)
        EXPECTED_INTERIOR_ANIMS["toe_brake_%s_%s" % (_side.lower(), _seat.lower())] = \
            "HANDLING_ToeBrake_%s_%s" % (_side, _seat)
REQUIRED_NODES = ["PROPELLER", "PROPELLER_Blur", "EFIS_Screen"]


def arg_value(name, default):
    return type(default)(ARGS[ARGS.index(name) + 1]) if name in ARGS else default


# ----------------------------------------------------------------------------
# export
# ----------------------------------------------------------------------------
def export():
    enable_msfs_exporter()
    scene = bpy.context.scene
    scene.frame_set(0)
    os.makedirs(MODEL_DIR, exist_ok=True)
    result = bpy.ops.msfs2024.multi_export_gltf(export_mode="PRESETS")
    if "FINISHED" not in result:
        raise RuntimeError("export failed: %s" % result)
    for name in ("A240", "A240_interior"):
        neutral_rest_pose(os.path.join(MODEL_DIR, name + ".gltf"))
    check_outputs()


def neutral_rest_pose(path):
    """The exporter stores each node's pose at the current frame as its rest transform, which
    for frame 0 means full control deflection. Clear the rotation of animated nodes so the
    rest pose is neutral, as before (every animated node is built in its neutral pose: flaps
    and brakes off, controls centred); the sim drives these nodes from the animations."""
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    targets = {c["target"]["node"] for a in doc.get("animations", []) for c in a["channels"]
               if c["target"]["path"] == "rotation"}
    changed = 0
    for i in targets:
        if doc["nodes"][i].pop("rotation", None) is not None:
            changed += 1
    if changed:
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(doc, f, indent=1)
    print("neutral rest pose on %d animated nodes" % changed)


def check_outputs():
    """Fail loudly if the files the sim relies on lost a node, an animation or the screen."""
    problems = []
    nodes_all, mats_all = set(), set()
    for name in ("A240", "A240_interior"):
        path = os.path.join(MODEL_DIR, name + ".gltf")
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
        nodes = [n["name"] for n in doc["nodes"]]
        nodes_all.update(nodes)
        mats_all.update(m["name"] for m in doc.get("materials", []))
        tris = 0
        for mesh in doc.get("meshes", []):
            for prim in mesh["primitives"]:
                tris += doc["accessors"][prim["indices"]]["count"] // 3
        anims = {}
        for a in doc.get("animations", []):
            targets = {doc["nodes"][c["target"]["node"]]["name"] for c in a["channels"]}
            anims.setdefault(a["name"], set()).update(targets)
        expected = EXPECTED_ANIMS if name == "A240" else EXPECTED_INTERIOR_ANIMS
        for anim, node in expected.items():
            if anims.get(anim) != {node}:
                problems.append("%s: animation %s targets %s, expected %s"
                                % (name, anim, sorted(anims.get(anim, [])), node))
        extra = set(anims) - set(expected)
        if extra:
            problems.append("%s: unexpected animations %s" % (name, sorted(extra)))
        print("%s.gltf: %d nodes, %d materials, %d triangles, %d animations"
              % (name, len(nodes), len(doc.get("materials", [])), tris, len(anims)))
    for n in REQUIRED_NODES:
        if n not in nodes_all:
            problems.append("node %s missing" % n)
    if "EFIS_Display" not in mats_all:
        problems.append("material EFIS_Display missing (the cockpit screen would be blank)")
    if problems:
        raise RuntimeError("export check failed:\n  " + "\n  ".join(problems))
    print("export check passed")


# ----------------------------------------------------------------------------
# preview rendering
# ----------------------------------------------------------------------------
def gltf_dir(v):
    """glTF-frame direction (+X left, +Y up, +Z nose) -> Blender."""
    return Vector((v[0], -v[2], v[1])).normalized()


VIEWS = [   # name, glTF view direction, glTF up, perspective
    ("three_quarter", (0.62, -0.38, -0.69), (0, 1, 0), True),
    ("side", (1, 0, 0), (0, 1, 0), False),
    ("top", (0, -1, 0), (0, 0, 1), False),
    ("front", (0, 0, -1), (0, 1, 0), False),
]
THUMBNAILS = [
    ("thumbnail.png", 720, 344, (0.62, -0.30, -0.72), True),
    ("thumbnail_button.png", 830, 260, (0.62, -0.30, -0.72), True),
    ("thumbnail_variation.png", 660, 300, (0.62, -0.30, -0.72), True),
    ("thumbnail_side.png", 930, 340, (1, 0, 0), False),
]
# sim-frame eye positions for close-up renders: (name, eye, look-at, lens mm, ground)
COCKPIT = [
    ("cockpit_pilot_view", (-0.28, 0.341, -0.149), (-0.18, 0.05, 0.55), 18, False),
    ("cockpit_panel", (0.0, 0.20, -0.05), (0.0, 0.10, 0.50), 22, False),
    ("cockpit_from_behind", (0.0, 0.42, -0.90), (0.0, -0.05, 0.40), 16, False),
    ("detail_nose", (1.35, 0.05, 3.10), (0.0, -0.25, 1.40), 40, True),
    ("detail_main_gear", (2.20, -0.55, 0.90), (0.75, -0.85, -0.30), 40, True),
    ("detail_tail", (1.60, 0.70, -1.60), (0.0, 0.35, -3.00), 35, True),
    ("detail_wingtip", (3.30, 0.30, 1.30), (4.55, 0.55, -0.30), 35, True),
    ("detail_cabin_side", (2.40, 0.30, 0.60), (0.40, 0.10, -0.20), 30, True),
]


def setup_render(samples):
    scene = bpy.context.scene
    for engine in ("BLENDER_EEVEE_NEXT", "BLENDER_EEVEE"):
        try:
            scene.render.engine = engine
            break
        except TypeError:
            continue
    if hasattr(scene, "eevee"):
        scene.eevee.taa_render_samples = samples
        for attr in ("use_shadows", "use_raytracing"):
            if hasattr(scene.eevee, attr):
                setattr(scene.eevee, attr, True)
    scene.render.film_transparent = False
    scene.view_settings.view_transform = "AgX" if "AgX" in [i.identifier for i in
                                                            scene.view_settings.bl_rna.properties["view_transform"].enum_items] else "Filmic"
    scene.view_settings.look = "None"
    world = bpy.data.worlds.new("preview_world")
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (0.62, 0.68, 0.76, 1.0)
    bg.inputs[1].default_value = 0.9
    scene.world = world
    sun = bpy.data.lights.new("preview_sun", "SUN")
    sun.energy = 3.2
    sun.angle = math.radians(3)
    sun_obj = bpy.data.objects.new("preview_sun", sun)
    scene.collection.objects.link(sun_obj)
    sun_obj.rotation_euler = (math.radians(50), math.radians(10), math.radians(-35))
    # ground plane for contact shadows
    me = bpy.data.meshes.new("preview_ground")
    s = 30.0
    gy = B((0, -1.12, 0)).z
    me.from_pydata([(-s, -s, gy), (s, -s, gy), (s, s, gy), (-s, s, gy)], [], [(0, 1, 2, 3)])
    gmat = bpy.data.materials.new("preview_ground")
    gmat.diffuse_color = (0.55, 0.57, 0.55, 1)
    if gmat.node_tree:
        bsdf = gmat.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            bsdf.inputs["Base Color"].default_value = (0.48, 0.50, 0.48, 1)
            bsdf.inputs["Roughness"].default_value = 0.9
    me.materials.append(gmat)
    ground = bpy.data.objects.new("preview_ground", me)
    scene.collection.objects.link(ground)
    for o in bpy.data.objects:
        if o.name.startswith("PROPELLER_Blur"):
            o.hide_render = True
    # In the sim the HTML display is painted onto EFIS_Display; in a render its huge
    # emissive value just blows out to white, so show a dark, softly lit screen instead.
    screen = bpy.data.materials.get("EFIS_Display")
    if screen and screen.node_tree:
        for node in screen.node_tree.nodes:
            if node.bl_idname == "ShaderNodeBsdfPrincipled":
                for link in list(screen.node_tree.links):
                    if link.to_node == node and link.to_socket.name in ("Base Color", "Emission Color",
                                                                         "Emission Strength"):
                        screen.node_tree.links.remove(link)
                node.inputs["Base Color"].default_value = (0.02, 0.05, 0.12, 1)
                node.inputs["Emission Color"].default_value = (0.10, 0.25, 0.55, 1)
                node.inputs["Emission Strength"].default_value = 1.5
    cam_data = bpy.data.cameras.new("preview_cam")
    cam = bpy.data.objects.new("preview_cam", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam
    return cam, ground


def model_points():
    pts = []
    for o in bpy.data.objects:
        if o.type == "MESH" and not o.name.startswith("preview_") and not o.hide_render:
            pts.extend(o.matrix_world @ Vector(c) for c in o.bound_box)
    return pts


def aim(cam, direction, up, perspective, width, height, margin=1.08):
    pts = model_points()
    d = direction.normalized()
    right = d.cross(up).normalized()
    upv = right.cross(d)
    xs = [p.dot(right) for p in pts]
    ys = [p.dot(upv) for p in pts]
    zs = [p.dot(d) for p in pts]
    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    center = right * cx + upv * cy + d * ((max(zs) + min(zs)) / 2)
    span_x, span_y = max(xs) - min(xs), max(ys) - min(ys)
    aspect = width / height
    ortho = max(span_x, span_y * aspect) * margin
    cam.data.type = "PERSP" if perspective else "ORTHO"
    cam.data.clip_start, cam.data.clip_end = 0.05, 200
    if perspective:
        cam.data.lens = 50
        fov = 2 * math.atan(cam.data.sensor_width / 2 / cam.data.lens)   # horizontal
        dist = (ortho / 2) / math.tan(fov / 2) + (max(zs) - min(zs)) / 2
        cam.location = center - d * dist
    else:
        cam.data.ortho_scale = ortho
        cam.location = center - d * 30


def look_from(cam, eye, target, lens):
    e, t = B(eye), B(target)
    d = (t - e).normalized()
    cam.data.type = "PERSP"
    cam.data.lens = lens
    cam.data.clip_start = 0.01
    cam.location = e
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def render_to(path, width, height):
    scene = bpy.context.scene
    scene.render.resolution_x, scene.render.resolution_y = width, height
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)
    print("rendered", path)


def render_previews(cockpit=False, samples=32):
    cam, ground = setup_render(samples)
    os.makedirs(PREVIEW_DIR, exist_ok=True)
    w, h = 820, 470
    paths = []
    for name, d, up, persp in VIEWS:
        direction = gltf_dir(d)
        upv = gltf_dir(up)
        ground.hide_render = name in ("top",)
        aim_with_up(cam, direction, upv, persp, w, h)
        path = os.path.join(PREVIEW_DIR, "A240_%s.png" % name)
        render_to(path, w, h)
        paths.append(path)
    ground.hide_render = False
    compose_sheet(paths, w, h, os.path.join(PREVIEW_DIR, "A240_sheet.png"))
    os.makedirs(THUMBNAIL_DIR, exist_ok=True)
    for fname, tw, th, d, persp in THUMBNAILS:
        aim_with_up(cam, gltf_dir(d), Vector((0, 0, 1)), persp, tw, th, margin=1.04)
        render_to(os.path.join(THUMBNAIL_DIR, fname), tw, th)
    if cockpit:
        for name, eye, target, lens, with_ground in COCKPIT:
            ground.hide_render = not with_ground
            look_from(cam, eye, target, lens)
            render_to(os.path.join(PREVIEW_DIR, name + ".png"), 1280, 800)


def aim_with_up(cam, direction, upv, persp, w, h, margin=1.08):
    aim(cam, direction, upv, persp, w, h, margin)
    # orient: forward = direction, up = upv
    right = direction.cross(upv).normalized()
    true_up = right.cross(direction).normalized()
    from mathutils import Matrix
    rot = Matrix((right, true_up, -direction)).transposed()
    cam.rotation_euler = rot.to_euler()


def compose_sheet(paths, w, h, out):
    import numpy as np
    sheet = np.ones((2 * h, 2 * w, 4), dtype=np.float32)
    for i, p in enumerate(paths):
        img = bpy.data.images.load(p)
        px = np.array(img.pixels[:], dtype=np.float32).reshape(h, w, 4)
        ox, oy = (i % 2) * w, (1 - i // 2) * h        # Blender images start bottom-left
        sheet[oy:oy + h, ox:ox + w] = px
        bpy.data.images.remove(img)
    out_img = bpy.data.images.new("sheet", 2 * w, 2 * h, alpha=True)
    out_img.pixels = sheet.ravel()
    out_img.filepath_raw = out
    out_img.file_format = "PNG"
    out_img.save()
    print("rendered", out)


def main():
    if "--no-export" not in ARGS:
        export()
    if "--previews" in ARGS or "--cockpit" in ARGS:
        render_previews(cockpit="--cockpit" in ARGS, samples=arg_value("--samples", 32))


if __name__ == "__main__":
    main()
