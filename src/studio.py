"""High-key product studio: infinity cove, soft boxes, render settings (Cycles, CPU)."""
import math
import bpy
import bmesh
from mathutils import Vector, Matrix

import geo


def clear_scene():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    for coll in (bpy.data.meshes, bpy.data.curves, bpy.data.materials, bpy.data.lights, bpy.data.cameras,
                 bpy.data.images, bpy.data.node_groups):
        for b in list(coll):
            if b.users == 0:
                coll.remove(b)
    for c in list(bpy.data.collections):
        bpy.data.collections.remove(c)


def new_collection(name, parent=None):
    c = bpy.data.collections.new(name)
    (parent or bpy.context.scene.collection).children.link(c)
    return c


def _floor_mat():
    m = bpy.data.materials.new("studio_floor")
    try:
        m.use_nodes = True
    except Exception:
        pass
    nt = m.node_tree
    b = nt.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (0.62, 0.62, 0.63, 1.0)
    b.inputs["Roughness"].default_value = 0.55
    b.inputs["Metallic"].default_value = 0.0
    b.inputs["Coat Weight"].default_value = 1.0
    b.inputs["Coat IOR"].default_value = 1.55
    # coat roughness: mostly smooth with very faint large-scale variation (polished epoxy)
    tc = nt.nodes.new("ShaderNodeTexCoord")
    nz = nt.nodes.new("ShaderNodeTexNoise")
    nz.inputs["Scale"].default_value = 1.6
    nz.inputs["Detail"].default_value = 4.0
    nt.links.new(tc.outputs["Object"], nz.inputs["Vector"])
    mr = nt.nodes.new("ShaderNodeMapRange")
    mr.inputs["From Min"].default_value = 0.3
    mr.inputs["From Max"].default_value = 0.7
    mr.inputs["To Min"].default_value = 0.045
    mr.inputs["To Max"].default_value = 0.085
    nt.links.new(nz.outputs["Fac"], mr.inputs["Value"])
    nt.links.new(mr.outputs["Result"], b.inputs["Coat Roughness"])
    return m


def _wall_mat():
    m = bpy.data.materials.new("studio_wall")
    try:
        m.use_nodes = True
    except Exception:
        pass
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (0.70, 0.70, 0.71, 1.0)
    b.inputs["Roughness"].default_value = 0.9
    return m


def build_cove(col, radius=9.0, fillet=2.2, height=7.0):
    """Circular infinity cove: flat floor disc, big fillet, cylindrical wall."""
    segs = 96
    prof = [(0.0, 0.0), (radius - fillet, 0.0)]
    for k in range(1, 13):
        a = (math.pi / 2) * k / 12
        prof.append((radius - fillet + fillet * math.sin(a), fillet - fillet * math.cos(a)))
    prof.append((radius, height))
    wall = geo.lathe("cove", prof, segs, "Z", col)
    # the inner disc gets the glossy floor material, the rest the matte wall
    wall.data.materials.append(_floor_mat())
    wall.data.materials.append(_wall_mat())
    rf = (radius - fillet) * 0.92
    for p in wall.data.polygons:
        c = p.center
        p.material_index = 0 if (c.z < 1e-4 and math.hypot(c.x, c.y) < rf) else 1
    # normals must face inward/up
    bm = bmesh.new()
    bm.from_mesh(wall.data)
    for f in bm.faces:
        c = f.calc_center_median()
        inward = Vector((-c.x, -c.y, 1.0 if c.z < 0.01 else 0.0))
        if f.normal.dot(inward) < 0:
            f.normal_flip()
    bm.to_mesh(wall.data)
    bm.free()
    wall.data.shade_smooth()
    return wall


def area_light(col, name, loc, target, size, power, color=(1, 1, 1), size_y=None, spread=math.pi):
    li = bpy.data.lights.new(name, "AREA")
    li.shape = "RECTANGLE" if size_y else "SQUARE"
    li.size = size
    if size_y:
        li.size_y = size_y
    li.energy = power
    li.color = color
    try:
        li.spread = spread
    except Exception:
        pass
    ob = bpy.data.objects.new(name, li)
    col.objects.link(ob)
    ob.location = loc
    d = Vector(target) - Vector(loc)
    ob.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    return ob


def build_lights(col, k=1.0):
    """Soft, high-key rig.  Returns dict name -> light object."""
    L = {}
    # big overhead soft box, slightly forward-left
    L["top"] = area_light(col, "L_top", (0.2, 0.3, 5.2), (0, 0, 0.4), 5.0, 2600 * k, (1.0, 0.985, 0.97), size_y=3.4)
    # long strip boxes left and right for the streak highlights on metal
    L["left"] = area_light(col, "L_left", (0.4, 4.6, 2.3), (0, 0, 0.6), 5.0, 1700 * k, (1.0, 0.99, 0.98), size_y=1.4)
    L["right"] = area_light(col, "L_right", (-0.6, -4.8, 1.9), (0, 0, 0.6), 5.0, 1150 * k, (0.97, 0.985, 1.0), size_y=1.2)
    # front fill, low and wide
    L["front"] = area_light(col, "L_front", (5.2, -0.8, 1.6), (0, 0, 0.55), 3.2, 650 * k, (1.0, 1.0, 1.0), size_y=1.6)
    # rear kicker
    L["rear"] = area_light(col, "L_rear", (-5.0, 1.6, 2.6), (0, 0, 0.6), 3.0, 800 * k, (1.0, 0.975, 0.94), size_y=1.4)
    return L


def build_world(strength=0.55, color=(0.80, 0.80, 0.81)):
    w = bpy.data.worlds.new("studio_world")
    bpy.context.scene.world = w
    try:
        w.use_nodes = True
    except Exception:
        pass
    bg = w.node_tree.nodes.get("Background")
    bg.inputs["Color"].default_value = (color[0], color[1], color[2], 1.0)
    bg.inputs["Strength"].default_value = strength
    return w


def setup_render(res=(1920, 1080), spp=64, threads=8, adaptive=0.03, min_spp=16, denoise=True, quality="HIGH"):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    c = sc.cycles
    c.device = "CPU"
    c.samples = spp
    c.use_adaptive_sampling = adaptive is not None
    if adaptive is not None:
        c.adaptive_threshold = adaptive
        c.adaptive_min_samples = min_spp
    c.use_denoising = denoise
    if denoise:
        c.denoiser = "OPENIMAGEDENOISE"
        c.denoising_input_passes = "RGB_ALBEDO_NORMAL"
        try:
            c.denoising_prefilter = "ACCURATE" if quality == "HIGH" else "FAST"
            c.denoising_quality = quality
        except Exception:
            pass
    c.max_bounces = 7
    c.diffuse_bounces = 2
    c.glossy_bounces = 6
    c.transmission_bounces = 8
    c.transparent_max_bounces = 8
    c.caustics_reflective = False
    c.caustics_refractive = False
    c.sample_clamp_indirect = 8.0
    c.blur_glossy = 0.3
    try:
        c.use_light_tree = False
    except Exception:
        pass
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = 100
    sc.render.threads_mode = "FIXED"
    sc.render.threads = threads
    sc.render.use_persistent_data = True
    sc.render.image_settings.file_format = "PNG"
    sc.render.image_settings.color_mode = "RGB"
    sc.render.image_settings.compression = 30
    sc.view_settings.view_transform = "AgX"
    try:
        sc.view_settings.look = "AgX - Medium High Contrast"
    except Exception:
        pass
    sc.view_settings.exposure = 0.0
    sc.render.film_transparent = False
    return sc


def add_camera(col, name="cam", lens=50.0, sensor=36.0):
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.sensor_width = sensor
    cd.clip_start = 0.01
    cd.clip_end = 100.0
    ob = bpy.data.objects.new(name, cd)
    col.objects.link(ob)
    bpy.context.scene.camera = ob
    return ob


def aim_camera(cam, loc, target, lens=None, fstop=None, focus=None, roll=0.0):
    cam.location = loc
    d = Vector(target) - Vector(loc)
    q = d.to_track_quat("-Z", "Y")
    if roll:
        q = q @ Matrix.Rotation(roll, 4, "Z").to_quaternion()
    cam.rotation_euler = q.to_euler()
    if lens:
        cam.data.lens = lens
    if fstop:
        cam.data.dof.use_dof = True
        cam.data.dof.aperture_fstop = fstop
        cam.data.dof.focus_distance = focus if focus else d.length
    else:
        cam.data.dof.use_dof = False


BG = (0.74, 0.745, 0.75)          # backdrop colour seen by the camera (scene linear)


def _floor_mat_open(fade0=3.2, fade1=8.0):
    """Glossy floor that dissolves into the backdrop colour with distance (infinite studio)."""
    m = _floor_mat()
    m.name = "studio_floor_open"
    nt = m.node_tree
    b = nt.nodes.get("Principled BSDF")
    out = [n for n in nt.nodes if n.type == "OUTPUT_MATERIAL"][0]
    geo_n = nt.nodes.new("ShaderNodeNewGeometry")
    ln = nt.nodes.new("ShaderNodeVectorMath")
    ln.operation = "LENGTH"
    nt.links.new(geo_n.outputs["Position"], ln.inputs[0])
    mr = nt.nodes.new("ShaderNodeMapRange")
    mr.interpolation_type = "SMOOTHERSTEP"
    mr.inputs["From Min"].default_value = fade0
    mr.inputs["From Max"].default_value = fade1
    nt.links.new(ln.outputs["Value"], mr.inputs["Value"])
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs["Color"].default_value = (BG[0], BG[1], BG[2], 1.0)
    em.inputs["Strength"].default_value = 1.0
    lp = nt.nodes.new("ShaderNodeLightPath")
    # only camera rays see the fade-to-backdrop emission; other rays keep the real floor
    mul = nt.nodes.new("ShaderNodeMath")
    mul.operation = "MULTIPLY"
    nt.links.new(mr.outputs["Result"], mul.inputs[0])
    nt.links.new(lp.outputs["Is Camera Ray"], mul.inputs[1])
    mix = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(mul.outputs[0], mix.inputs["Fac"])
    nt.links.new(b.outputs["BSDF"], mix.inputs[1])
    nt.links.new(em.outputs["Emission"], mix.inputs[2])
    nt.links.new(mix.outputs["Shader"], out.inputs["Surface"])
    return m


PANELS = [  # (azimuth deg, half width deg, strength)
    (58.0, 20.0, 1.45), (148.0, 16.0, 0.8), (205.0, 18.0, 1.2), (300.0, 18.0, 1.0), (352.0, 9.0, 0.7),
    (18.0, 6.0, 1.0), (102.0, 7.0, 0.9), (252.0, 8.0, 0.9),
]


def build_world_open(env=0.11, top=0.07, panels=PANELS, panel_gain=1.0):
    """Camera sees the flat backdrop colour; lighting / reflections see a grey dome with soft vertical
    light panels around the horizon (they give brushed metal gradients)."""
    w = bpy.data.worlds.new("studio_world_open")
    bpy.context.scene.world = w
    try:
        w.use_nodes = True
    except Exception:
        pass
    nt = w.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputWorld")
    bg_cam = nt.nodes.new("ShaderNodeBackground")
    bg_cam.inputs["Color"].default_value = (BG[0], BG[1], BG[2], 1.0)
    bg_cam.inputs["Strength"].default_value = 1.0
    bg_env = nt.nodes.new("ShaderNodeBackground")
    tc = nt.nodes.new("ShaderNodeTexCoord")
    nrm = nt.nodes.new("ShaderNodeVectorMath")
    nrm.operation = "NORMALIZE"
    nt.links.new(tc.outputs["Generated"], nrm.inputs[0])
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    nt.links.new(nrm.outputs["Vector"], sep.inputs[0])

    def math_(op, a, b=None, clamp=False):
        n = nt.nodes.new("ShaderNodeMath")
        n.operation = op
        n.use_clamp = clamp
        for i, v in enumerate((a, b)):
            if v is None:
                continue
            if isinstance(v, (int, float)):
                n.inputs[i].default_value = v
            else:
                nt.links.new(v, n.inputs[i])
        return n.outputs[0]

    def maprange(v, a, b, c, d, smooth=True):
        n = nt.nodes.new("ShaderNodeMapRange")
        if smooth:
            n.interpolation_type = "SMOOTHERSTEP"
        n.inputs["From Min"].default_value = a
        n.inputs["From Max"].default_value = b
        n.inputs["To Min"].default_value = c
        n.inputs["To Max"].default_value = d
        nt.links.new(v, n.inputs["Value"])
        return n.outputs["Result"]

    total = maprange(sep.outputs["Z"], 0.0, 1.0, env, top, smooth=False)
    emask = math_("MULTIPLY", maprange(sep.outputs["Z"], -0.02, 0.10, 0.0, 1.0),
                  maprange(sep.outputs["Z"], 0.55, 0.85, 1.0, 0.0))
    hl = math_("SQRT", math_("ADD", math_("MULTIPLY", sep.outputs["X"], sep.outputs["X"]),
                             math_("MULTIPLY", sep.outputs["Y"], sep.outputs["Y"])))
    hl = math_("MAXIMUM", hl, 1e-4)
    for (az, hw, k) in panels:
        a = math.radians(az)
        dot = math_("ADD", math_("MULTIPLY", sep.outputs["X"], math.cos(a)), math_("MULTIPLY", sep.outputs["Y"], math.sin(a)))
        cosd = math_("DIVIDE", dot, hl)
        band = maprange(cosd, math.cos(math.radians(hw)), math.cos(math.radians(hw * 0.25)), 0.0, k * panel_gain)
        total = math_("ADD", total, math_("MULTIPLY", band, emask))
    bg_env.inputs["Color"].default_value = (0.8, 0.8, 0.81, 1.0)
    nt.links.new(total, bg_env.inputs["Strength"])
    lp = nt.nodes.new("ShaderNodeLightPath")
    mix = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(lp.outputs["Is Camera Ray"], mix.inputs["Fac"])
    nt.links.new(bg_env.outputs["Background"], mix.inputs[1])
    nt.links.new(bg_cam.outputs["Background"], mix.inputs[2])
    nt.links.new(mix.outputs["Shader"], out.inputs["Surface"])
    return w


def build_lights_open(col, k=1.0):
    L = {}
    L["top"] = area_light(col, "L_top", (0.1, 0.2, 5.0), (0, 0, 0.4), 5.2, 70 * k, (1.0, 0.985, 0.97), size_y=3.2)
    L["key"] = area_light(col, "L_key", (3.2, 3.6, 3.4), (0, 0, 0.5), 2.6, 34 * k, (1.0, 0.99, 0.975), size_y=1.6)
    L["rim"] = area_light(col, "L_rim", (-4.2, -2.6, 2.6), (0, 0, 0.6), 2.4, 30 * k, (0.97, 0.985, 1.0), size_y=1.2)
    return L


def build_open(k=1.0, env=0.11, floor_emit=0.80, ao_dist=0.9):
    """Cheap infinite studio: huge floor disc fading into the backdrop + soft boxes + panel dome.
    The floor glows towards the camera only (high-key look) and is darkened by ambient occlusion so
    objects still sit on it; reflections see the darker, purely lit floor (contrast for the metal)."""
    col = new_collection("studio")
    bm = bmesh.new()
    bmesh.ops.create_circle(bm, cap_ends=True, cap_tris=False, segments=96, radius=60.0)
    floor = geo.obj_from_bm("floor", bm, col)
    if floor.data.polygons[0].normal.z < 0:
        floor.data.flip_normals()
    fmat = _floor_mat_open()
    nt = fmat.node_tree
    b = nt.nodes.get("Principled BSDF")
    b.inputs["Emission Color"].default_value = (BG[0], BG[1], BG[2], 1.0)
    ao = nt.nodes.new("ShaderNodeAmbientOcclusion")
    ao.inputs["Distance"].default_value = ao_dist
    ao.samples = 4
    lp = nt.nodes.new("ShaderNodeLightPath")
    pw = nt.nodes.new("ShaderNodeMath")
    pw.operation = "POWER"
    nt.links.new(ao.outputs["AO"], pw.inputs[0])
    pw.inputs[1].default_value = 1.6
    m1 = nt.nodes.new("ShaderNodeMath")
    m1.operation = "MULTIPLY"
    nt.links.new(pw.outputs[0], m1.inputs[0])
    nt.links.new(lp.outputs["Is Camera Ray"], m1.inputs[1])
    m2 = nt.nodes.new("ShaderNodeMath")
    m2.operation = "MULTIPLY"
    nt.links.new(m1.outputs[0], m2.inputs[0])
    m2.inputs[1].default_value = floor_emit
    nt.links.new(m2.outputs[0], b.inputs["Emission Strength"])
    floor.data.materials.append(fmat)
    lights = build_lights_open(col, k)
    build_world_open(env)
    return {"col": col, "cove": floor, "floor": floor, "lights": lights}



def build(k=0.10, world=0.30):
    col = new_collection("studio")
    cove = build_cove(col)
    lights = build_lights(col, k)
    build_world(world)
    return {"col": col, "cove": cove, "lights": lights}
