"""Shared material library (Cycles, Principled BSDF).  Use  mats.get("name").

Brushed materials brush along the OBJECT's local X axis ("*_x" / plates) or
concentrically around the object's local Z axis ("*_turned").  Rotate / orient the
object (not the mesh data) so the brushing runs where you want it, or pick the other variant.

Names
  alu_plate      brushed aluminium sheet (cube panels, bodywork), linear brushing along local X
  alu_brushed    same finish, a little brighter / finer (machined brackets, levers), along local X
  alu_turned     lathe-turned aluminium, concentric brushing around local Z (hubs, caps, rims)
  alu_machined   bright milled aluminium, no visible direction
  alu_cast       sand-cast aluminium, satin, fine grain
  ano_orange     orange anodised aluminium (frame, linkages), light brushing along local X
  ano_orange_turned  orange anodised, turned around local Z
  ano_black      black anodised aluminium
  ano_titan      titanium-grey anodised (fork outer tubes)
  chrome         polished chrome / stanchions
  steel_polished polished stainless (spokes, axle ends)
  steel_brushed  brushed stainless along local X (exhaust headers)
  steel_turned   stainless, concentric around local Z (brake discs, washers)
  steel_dark     black-oxide fasteners
  titanium       titanium exhaust can (warm grey, soft brushing along local X)
  ti_blued       heat-coloured titanium (straw/purple/blue) for header bends and can tip
  engine_black   satin black crinkle paint on castings
  engine_fin     black fins with bare machined edges look (dark satin metal)
  rubber_tire    tyre rubber
  rubber         black rubber (grips, hoses, gaiters)
  plastic_black  satin black plastic
  leather        black seat leather with grain
  stitch         light grey thread
  glass          clear glass
  glass_lens     ribbed-looking headlight glass (slightly rough)
  mirror         perfect reflector
  emit_head      warm white emitter, strength scaled by object custom property "emit" (default 1)
  emit_red       red LED emitter, strength scaled by object custom property "emit"
  amber_fluid    amber brake fluid / indicator lens
  gauge_face     black dial face
  white_paint    white markings
  carbon         clear-coated carbon fibre weave        braid      braided stainless hose
  brass, copper  fittings                               zinc       dull zinc-plated hardware
  ano_blue, ano_red, ano_gold   small anodised accents (AN fittings, adjusters, valve caps) - use sparingly
  glass_red      red lens plastic
"""
import bpy

_cache = {}


def _base(name):
    m = bpy.data.materials.new(name)
    try:
        m.use_nodes = True
    except Exception:
        pass
    nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    out.location = (600, 0)
    b = nt.nodes.new("ShaderNodeBsdfPrincipled")
    b.location = (300, 0)
    nt.links.new(b.outputs["BSDF"], out.inputs["Surface"])
    return m, nt, b


def _set(b, **kw):
    names = {
        "color": "Base Color", "metallic": "Metallic", "rough": "Roughness", "ior": "IOR",
        "aniso": "Anisotropic", "aniso_rot": "Anisotropic Rotation", "trans": "Transmission Weight",
        "coat": "Coat Weight", "coat_rough": "Coat Roughness", "sheen": "Sheen Weight",
        "sheen_rough": "Sheen Roughness", "spec": "Specular IOR Level", "emis_color": "Emission Color",
        "emis": "Emission Strength", "alpha": "Alpha",
    }
    for k, v in kw.items():
        inp = b.inputs.get(names[k])
        if inp is None:
            continue
        if k in ("color", "emis_color") and len(v) == 3:
            v = (v[0], v[1], v[2], 1.0)
        inp.default_value = v


def _N(nt, typ, loc=(0, 0), **props):
    n = nt.nodes.new(typ)
    n.location = loc
    for k, v in props.items():
        setattr(n, k, v)
    return n


def _math(nt, op, a, b=None, loc=(0, 0), clamp=False):
    n = _N(nt, "ShaderNodeMath", loc, operation=op)
    n.use_clamp = clamp
    for i, v in enumerate((a, b)):
        if v is None:
            continue
        if isinstance(v, (int, float)):
            n.inputs[i].default_value = v
        else:
            nt.links.new(v, n.inputs[i])
    return n.outputs[0]


def _noise(nt, vec, scale, detail=2.0, rough=0.5, loc=(0, 0)):
    n = _N(nt, "ShaderNodeTexNoise", loc)
    n.inputs["Scale"].default_value = scale
    n.inputs["Detail"].default_value = detail
    n.inputs["Roughness"].default_value = rough
    if vec is not None:
        nt.links.new(vec, n.inputs["Vector"])
    return n.outputs["Fac"]


def _mapping(nt, vec, scale, loc=(0, 0)):
    n = _N(nt, "ShaderNodeMapping", loc)
    n.inputs["Scale"].default_value = scale
    nt.links.new(vec, n.inputs["Vector"])
    return n.outputs["Vector"]


def _bump(nt, b, height, strength=0.05, distance=0.0005):
    n = _N(nt, "ShaderNodeBump", (60, -420))
    n.inputs["Strength"].default_value = strength
    n.inputs["Distance"].default_value = distance
    nt.links.new(height, n.inputs["Height"])
    nt.links.new(n.outputs["Normal"], b.inputs["Normal"])
    return n


def _brush_linear(nt, b, rough, rough_var=0.10, aniso=0.72, bump=0.035, fine=1500.0, coarse=120.0, length=2.5):
    """Brushing along object local X."""
    tc = _N(nt, "ShaderNodeTexCoord", (-900, 0))
    obj = tc.outputs["Object"]
    f = _noise(nt, _mapping(nt, obj, (length, fine, fine), (-700, 120)), 1.0, 3.0, 0.6, (-500, 120))
    c = _noise(nt, _mapping(nt, obj, (length * 0.4, coarse, coarse), (-700, -120)), 1.0, 2.0, 0.5, (-500, -120))
    mix = _math(nt, "ADD", _math(nt, "MULTIPLY", c, rough_var, (-300, -120)),
                _math(nt, "MULTIPLY", f, rough_var * 0.5, (-300, 120)), (-150, 0))
    r = _math(nt, "ADD", mix, rough - rough_var * 0.75, (0, 0), clamp=True)
    nt.links.new(r, b.inputs["Roughness"])
    _bump(nt, b, f, bump, 0.0003)
    b.inputs["Anisotropic"].default_value = aniso
    b.inputs["Anisotropic Rotation"].default_value = 0.25
    vt = _N(nt, "ShaderNodeVectorTransform", (-200, -620), vector_type="VECTOR",
            convert_from="OBJECT", convert_to="WORLD")
    vt.inputs["Vector"].default_value = (1.0, 0.0, 0.0)
    nt.links.new(vt.outputs["Vector"], b.inputs["Tangent"])
    return f, c


def _brush_turned(nt, b, rough, rough_var=0.10, aniso=0.75, bump=0.03, fine=1400.0):
    """Concentric (lathe) brushing around object local Z."""
    tc = _N(nt, "ShaderNodeTexCoord", (-1100, 0))
    sep = _N(nt, "ShaderNodeSeparateXYZ", (-950, 0))
    nt.links.new(tc.outputs["Object"], sep.inputs[0])
    rr = _math(nt, "SQRT", _math(nt, "ADD", _math(nt, "POWER", sep.outputs[0], 2.0, (-800, 100)),
                                 _math(nt, "POWER", sep.outputs[1], 2.0, (-800, -60)), (-650, 20)), None, (-520, 20))
    comb = _N(nt, "ShaderNodeCombineXYZ", (-380, 0))
    nt.links.new(rr, comb.inputs[0])
    nt.links.new(sep.outputs[2], comb.inputs[2])
    f = _noise(nt, _mapping(nt, comb.outputs[0], (fine, 1.0, fine), (-220, 140)), 1.0, 3.0, 0.6, (-60, 300))
    c = _noise(nt, _mapping(nt, comb.outputs[0], (fine * 0.08, 1.0, fine * 0.08), (-220, -140)), 1.0, 2.0, 0.5, (-60, -300))
    mix = _math(nt, "ADD", _math(nt, "MULTIPLY", c, rough_var), _math(nt, "MULTIPLY", f, rough_var * 0.5))
    r = _math(nt, "ADD", mix, rough - rough_var * 0.75, clamp=True)
    nt.links.new(r, b.inputs["Roughness"])
    _bump(nt, b, f, bump, 0.0003)
    b.inputs["Anisotropic"].default_value = aniso
    b.inputs["Anisotropic Rotation"].default_value = 0.25
    tg = _N(nt, "ShaderNodeTangent", (-200, -620), direction_type="RADIAL", axis="Z")
    nt.links.new(tg.outputs["Tangent"], b.inputs["Tangent"])


def _grain(nt, b, rough, scale, strength, rough_var=0.06, detail=4.0, distance=0.0004):
    tc = _N(nt, "ShaderNodeTexCoord", (-700, 0))
    f = _noise(nt, tc.outputs["Object"], scale, detail, 0.6, (-500, 0))
    r = _math(nt, "ADD", _math(nt, "MULTIPLY", f, rough_var), rough - rough_var * 0.5, clamp=True)
    nt.links.new(r, b.inputs["Roughness"])
    _bump(nt, b, f, strength, distance)


def _emit_scaled(nt, b, color, strength):
    att = _N(nt, "ShaderNodeAttribute", (-300, -300), attribute_type="OBJECT", attribute_name="emit")
    # an object without the custom property returns 0 -> fall back to 1 via "alpha" output trick is unreliable,
    # so objects using these materials must define obj["emit"] (mats.set_emit does it).
    s = _math(nt, "MULTIPLY", att.outputs["Fac"], strength, (-100, -300))
    nt.links.new(s, b.inputs["Emission Strength"])
    b.inputs["Emission Color"].default_value = (color[0], color[1], color[2], 1.0)


def set_emit(obj, value=1.0):
    obj["emit"] = float(value)


ALU = (0.83, 0.84, 0.85)


def _build(name):
    m, nt, b = _base(name)
    if name == "alu_plate":
        _set(b, color=(0.80, 0.81, 0.825), metallic=1.0)
        f_, c_ = _brush_linear(nt, b, rough=0.33, rough_var=0.085, aniso=0.70, bump=0.035)
        # brushing also modulates the colour a little so it reads from a distance
        tcc = _N(nt, "ShaderNodeTexCoord", (-900, 500))
        m_ = _noise(nt, _mapping(nt, tcc.outputs["Object"], (1.2, 420.0, 420.0), (-700, 500)), 1.0, 2.0, 0.55, (-500, 500))
        mixv = _math(nt, "ADD", _math(nt, "MULTIPLY", c_, 0.10), _math(nt, "MULTIPLY", m_, 0.10))
        val = _math(nt, "ADD", mixv, 0.90, clamp=True)
        mul = _N(nt, "ShaderNodeMixRGB", (0, 420))
        mul.blend_type = "MULTIPLY"
        mul.inputs[0].default_value = 1.0
        mul.inputs[1].default_value = (0.82, 0.83, 0.845, 1.0)
        cmb = _N(nt, "ShaderNodeCombineColor", (-150, 420)) if hasattr(bpy.types, "ShaderNodeCombineColor") else None
        if cmb is not None:
            for k_ in range(3):
                nt.links.new(val, cmb.inputs[k_])
            nt.links.new(cmb.outputs[0], mul.inputs[2])
            nt.links.new(mul.outputs[0], b.inputs["Base Color"])
    elif name == "alu_brushed":
        _set(b, color=(0.86, 0.865, 0.87), metallic=1.0)
        _brush_linear(nt, b, rough=0.26, rough_var=0.08, aniso=0.65, bump=0.025, fine=2200.0)
    elif name == "alu_turned":
        _set(b, color=(0.86, 0.865, 0.87), metallic=1.0)
        _brush_turned(nt, b, rough=0.24, rough_var=0.08)
    elif name == "alu_machined":
        _set(b, color=(0.88, 0.885, 0.89), metallic=1.0)
        _grain(nt, b, 0.20, 900.0, 0.01, 0.05)
    elif name == "alu_cast":
        _set(b, color=(0.70, 0.705, 0.71), metallic=1.0)
        _grain(nt, b, 0.50, 1400.0, 0.12, 0.10)
    elif name == "ano_orange":
        _set(b, color=(0.86, 0.27, 0.025), metallic=1.0)
        _brush_linear(nt, b, rough=0.30, rough_var=0.07, aniso=0.45, bump=0.012, fine=2500.0)
    elif name == "ano_orange_turned":
        _set(b, color=(0.86, 0.27, 0.025), metallic=1.0)
        _brush_turned(nt, b, rough=0.28, rough_var=0.07, aniso=0.5, bump=0.012)
    elif name == "ano_black":
        _set(b, color=(0.035, 0.035, 0.038), metallic=1.0)
        _grain(nt, b, 0.34, 1200.0, 0.01, 0.06)
    elif name == "ano_titan":
        _set(b, color=(0.42, 0.40, 0.38), metallic=1.0)
        _brush_turned(nt, b, rough=0.26, rough_var=0.06, aniso=0.5, bump=0.01)
    elif name == "chrome":
        _set(b, color=(0.93, 0.93, 0.94), metallic=1.0, rough=0.035)
    elif name == "steel_polished":
        _set(b, color=(0.78, 0.78, 0.79), metallic=1.0, rough=0.09)
    elif name == "steel_brushed":
        _set(b, color=(0.66, 0.66, 0.67), metallic=1.0)
        _brush_linear(nt, b, rough=0.27, rough_var=0.08, aniso=0.6, bump=0.02, fine=2400.0)
    elif name == "steel_turned":
        _set(b, color=(0.62, 0.62, 0.63), metallic=1.0)
        _brush_turned(nt, b, rough=0.25, rough_var=0.09, aniso=0.8, bump=0.03)
    elif name == "steel_dark":
        _set(b, color=(0.07, 0.07, 0.075), metallic=1.0)
        _grain(nt, b, 0.38, 1500.0, 0.01, 0.08)
    elif name == "titanium":
        _set(b, color=(0.60, 0.565, 0.53), metallic=1.0)
        _brush_linear(nt, b, rough=0.30, rough_var=0.08, aniso=0.55, bump=0.02, fine=2000.0)
    elif name == "ti_blued":
        # heat colours driven by a slow noise along the part
        _set(b, metallic=1.0, rough=0.24)
        tc = _N(nt, "ShaderNodeTexCoord", (-900, 200))
        f = _noise(nt, tc.outputs["Object"], 9.0, 2.0, 0.5, (-700, 200))
        ramp = _N(nt, "ShaderNodeValToRGB", (-450, 200))
        cr = ramp.color_ramp
        cr.elements[0].position = 0.30
        cr.elements[0].color = (0.60, 0.52, 0.40, 1)
        cr.elements[1].position = 0.72
        cr.elements[1].color = (0.30, 0.36, 0.56, 1)
        e = cr.elements.new(0.50)
        e.color = (0.52, 0.38, 0.46, 1)
        nt.links.new(f, ramp.inputs["Fac"])
        nt.links.new(ramp.outputs["Color"], b.inputs["Base Color"])
    elif name == "engine_black":
        _set(b, color=(0.016, 0.016, 0.017), metallic=0.0, spec=0.5)
        _grain(nt, b, 0.46, 2600.0, 0.22, 0.10, detail=2.0, distance=0.0003)
    elif name == "engine_fin":
        _set(b, color=(0.045, 0.045, 0.048), metallic=1.0)
        _grain(nt, b, 0.40, 1800.0, 0.05, 0.08)
    elif name == "rubber_tire":
        _set(b, color=(0.014, 0.014, 0.015), metallic=0.0, spec=0.35, sheen=0.15, sheen_rough=0.5)
        _grain(nt, b, 0.72, 2200.0, 0.10, 0.10, detail=3.0, distance=0.0003)
    elif name == "rubber":
        _set(b, color=(0.02, 0.02, 0.021), metallic=0.0, spec=0.4)
        _grain(nt, b, 0.60, 1800.0, 0.06, 0.08)
    elif name == "plastic_black":
        _set(b, color=(0.025, 0.025, 0.027), metallic=0.0, spec=0.5)
        _grain(nt, b, 0.36, 2500.0, 0.03, 0.05)
    elif name == "leather":
        _set(b, color=(0.02, 0.019, 0.018), metallic=0.0, spec=0.45)
        tc = _N(nt, "ShaderNodeTexCoord", (-900, 0))
        v = _N(nt, "ShaderNodeTexVoronoi", (-650, 0))
        v.feature = "DISTANCE_TO_EDGE"
        v.inputs["Scale"].default_value = 520.0
        nt.links.new(tc.outputs["Object"], v.inputs["Vector"])
        n2 = _noise(nt, tc.outputs["Object"], 60.0, 3.0, 0.6, (-650, -300))
        h = _math(nt, "ADD", _math(nt, "MULTIPLY", v.outputs["Distance"], 1.6, (-420, 0), clamp=True),
                  _math(nt, "MULTIPLY", n2, 0.5, (-420, -300)), (-250, -100))
        r = _math(nt, "ADD", _math(nt, "MULTIPLY", n2, 0.12), 0.47, clamp=True)
        nt.links.new(r, b.inputs["Roughness"])
        _bump(nt, b, h, 0.35, 0.0004)
    elif name == "stitch":
        _set(b, color=(0.55, 0.54, 0.52), metallic=0.0, rough=0.7)
    elif name == "glass":
        _set(b, color=(1, 1, 1), metallic=0.0, rough=0.0, trans=1.0, ior=1.5)
    elif name == "glass_lens":
        _set(b, color=(1, 1, 1), metallic=0.0, rough=0.03, trans=1.0, ior=1.5)
    elif name == "mirror":
        _set(b, color=(0.96, 0.96, 0.96), metallic=1.0, rough=0.02)
    elif name == "emit_head":
        _set(b, color=(0.9, 0.86, 0.78), metallic=0.0, rough=0.3)
        _emit_scaled(nt, b, (1.0, 0.86, 0.62), 22.0)
    elif name == "emit_red":
        _set(b, color=(0.35, 0.01, 0.005), metallic=0.0, rough=0.3)
        _emit_scaled(nt, b, (1.0, 0.045, 0.02), 14.0)
    elif name == "amber_fluid":
        _set(b, color=(0.85, 0.50, 0.05), metallic=0.0, rough=0.05, trans=0.9, ior=1.45)
    elif name == "gauge_face":
        _set(b, color=(0.012, 0.012, 0.013), metallic=0.0, rough=0.5)
    elif name == "carbon":
        # 2x2 twill-ish carbon fibre: checker switches the anisotropy direction, clear coat on top
        _set(b, color=(0.012, 0.012, 0.014), metallic=0.0, rough=0.32, spec=0.6, coat=1.0, coat_rough=0.06)
        tc = _N(nt, "ShaderNodeTexCoord", (-900, 0))
        mp = _mapping(nt, tc.outputs["Object"], (420.0, 420.0, 420.0), (-700, 0))
        ck = _N(nt, "ShaderNodeTexChecker", (-500, 0))
        ck.inputs["Scale"].default_value = 1.0
        nt.links.new(mp, ck.inputs["Vector"])
        wv = _N(nt, "ShaderNodeTexWave", (-500, -260))
        wv.inputs["Scale"].default_value = 1800.0
        wv.inputs["Distortion"].default_value = 0.0
        nt.links.new(tc.outputs["Object"], wv.inputs["Vector"])
        r = _math(nt, "ADD", _math(nt, "MULTIPLY", ck.outputs["Fac"], 0.16), 0.22, clamp=True)
        nt.links.new(r, b.inputs["Roughness"])
        col = _N(nt, "ShaderNodeMixRGB", (-250, 200))
        col.inputs[1].default_value = (0.010, 0.010, 0.012, 1)
        col.inputs[2].default_value = (0.035, 0.035, 0.040, 1)
        nt.links.new(ck.outputs["Fac"], col.inputs[0])
        nt.links.new(col.outputs[0], b.inputs["Base Color"])
        _bump(nt, b, wv.outputs["Fac"], 0.06, 0.0002)
    elif name == "braid":
        # braided stainless hose
        _set(b, color=(0.62, 0.62, 0.64), metallic=1.0, rough=0.34)
        tc = _N(nt, "ShaderNodeTexCoord", (-900, 0))
        ck = _N(nt, "ShaderNodeTexChecker", (-500, 0))
        ck.inputs["Scale"].default_value = 900.0
        nt.links.new(tc.outputs["Object"], ck.inputs["Vector"])
        r = _math(nt, "ADD", _math(nt, "MULTIPLY", ck.outputs["Fac"], 0.22), 0.24, clamp=True)
        nt.links.new(r, b.inputs["Roughness"])
        _bump(nt, b, ck.outputs["Fac"], 0.5, 0.0003)
    elif name == "brass":
        _set(b, color=(0.80, 0.60, 0.26), metallic=1.0)
        _grain(nt, b, 0.24, 1200.0, 0.01, 0.06)
    elif name == "copper":
        _set(b, color=(0.86, 0.46, 0.30), metallic=1.0)
        _grain(nt, b, 0.26, 1200.0, 0.01, 0.06)
    elif name == "ano_blue":
        _set(b, color=(0.05, 0.16, 0.62), metallic=1.0)
        _grain(nt, b, 0.28, 1200.0, 0.01, 0.05)
    elif name == "ano_red":
        _set(b, color=(0.62, 0.035, 0.03), metallic=1.0)
        _grain(nt, b, 0.28, 1200.0, 0.01, 0.05)
    elif name == "ano_gold":
        _set(b, color=(0.83, 0.62, 0.22), metallic=1.0)
        _grain(nt, b, 0.26, 1200.0, 0.01, 0.05)
    elif name == "glass_red":
        _set(b, color=(0.75, 0.03, 0.02), metallic=0.0, rough=0.06, trans=0.85, ior=1.49)
    elif name == "zinc":
        _set(b, color=(0.58, 0.60, 0.62), metallic=1.0)
        _grain(nt, b, 0.42, 900.0, 0.03, 0.12)
    elif name == "white_paint":
        _set(b, color=(0.85, 0.85, 0.83), metallic=0.0, rough=0.45)
    else:
        raise KeyError("unknown material " + name)
    return m


def get(name):
    m = _cache.get(name)
    if m is None or m.name not in bpy.data.materials:
        m = _build(name)
        _cache[name] = m
    return m


def names():
    return ["alu_plate", "alu_brushed", "alu_turned", "alu_machined", "alu_cast", "ano_orange",
            "ano_orange_turned", "ano_black", "ano_titan", "chrome", "steel_polished", "steel_brushed",
            "steel_turned", "steel_dark", "titanium", "ti_blued", "engine_black", "engine_fin",
            "rubber_tire", "rubber", "plastic_black", "leather", "stitch", "glass", "glass_lens", "mirror",
            "emit_head", "emit_red", "amber_fluid", "gauge_face", "white_paint", "carbon", "braid", "brass",
            "copper", "ano_blue", "ano_red", "ano_gold", "glass_red", "zinc"]
