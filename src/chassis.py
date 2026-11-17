"""Chassis geometry (frame, head carrier, folding swingarm, shock, seat carriage, arms).

Builders create meshes either in FINAL-pose world coordinates ("world" builders, to be attached with
Node.attach while the rig is in its final pose) or in node-local coordinates ("local" builders).
"""
import math
import bpy
import bmesh
from mathutils import Vector, Matrix

import common as C
import geo
import mats
from rigbase import T, R, frame, aim

SA_ANGLE = math.asin((C.PIVOT[2] - C.RW_C[2]) / (C.LINK1 + C.LINK2))   # swingarm droop at ride height
B_HINGE = (C.PIVOT[0], C.ARM_Y, C.PIVOT[2])
TUBE_R = 0.0105

# engine mount bosses (y is refined from the engine agent's report)
BOSS = dict(head=(0.10, 0.164, 0.615), rear=(-0.19, 0.143, 0.52), front=(0.13, 0.143, 0.33))

SHOCK_TOP = (-0.050, 0.215, 0.672)       # frame side eye
SHOCK_BOT = (-0.275, 0.215, 0.520)       # bell-crank eye (moves with the swingarm pitch node)
CARR_Y = 0.10
CARR_Z = 0.775
CARR_X = (-1.06, -0.30)
SEAT_X = (-0.745, -0.32)


def M(n):
    return mats.get(n)


def orange_tube(name, pts, col, r=TUBE_R, fillet=0.0):
    return geo.tube(name, pts, r, 20, fillet=fillet, col=col, mat=M("ano_orange"))


def lug(name, p, col, axis="Y", r=0.0165, h=0.030, mat="ano_orange", bolt_d=0.008, both=True):
    """Machined cylindrical node at p with bolt heads on the faces."""
    o = geo.cyl(name, r, h, 32, bevel=0.0015, axis=axis, col=col, mat=M(mat), loc=p)
    out = [o]
    for sgn in ((1, -1) if both else (1,)):
        b = geo.bolt(name + "_b", bolt_d, col=col, mat=M("steel_polished"), washer=True)
        if axis == "Y":
            b.rotation_euler = (-math.pi / 2 * sgn, 0, 0)
            b.location = (p[0], p[1] + sgn * h / 2, p[2])
        elif axis == "X":
            b.rotation_euler = (0, math.pi / 2 * sgn, 0)
            b.location = (p[0] + sgn * h / 2, p[1], p[2])
        else:
            b.rotation_euler = (0 if sgn > 0 else math.pi, 0, 0)
            b.location = (p[0], p[1], p[2] + sgn * h / 2)
        out.append(b)
    return out


def plate_xz(name, pts, y, th, col, mat="ano_orange", bevel=0.0015, holes=None):
    """Plate in the XZ plane (list of (x, z)), centred at y, thickness th. holes: [(x, z, r)]."""
    o = geo.prism(name, [(p[0], p[1]) for p in pts], th, bevel=bevel, plane="XZ", col=col, mat=M(mat))
    geo.transform_mesh(o, loc=(0, y, 0))
    if holes:
        cut = []
        for (hx, hz, hr) in holes:
            c = geo.cyl(name + "_cut", hr, th * 3, 32, axis="Y", col=col, loc=(hx, y, hz))
            cut.append(c)
        cj = geo.join(cut, name + "_cut")
        m = o.modifiers.new("holes", "BOOLEAN")
        m.operation = "DIFFERENCE"
        m.object = cj
        try:
            m.solver = "EXACT"
        except Exception:
            pass
        geo.apply_mods(o)
        bpy.data.objects.remove(cj, do_unlink=True)
        geo.set_smooth(o, 35)
        geo.bevel_mod(o, 0.0008, 2, 35)
    return o


# ----------------------------------------------------------------------------- frame (world, final)
def build_frame(col):
    objs = []
    for s in (1, -1):
        sg = "L" if s > 0 else "R"
        A = (0.250, s * 0.118, 0.745)
        B = (-0.02, s * 0.118, 0.745)
        Cc = (-0.33, s * 0.125, 0.745)
        HB = (BOSS["head"][0], s * (BOSS["head"][1] + 0.016), BOSS["head"][2])
        RB = (BOSS["rear"][0], s * (BOSS["rear"][1] + 0.016), BOSS["rear"][2])
        objs.append(orange_tube("rail_" + sg, [A, B, Cc], col))
        objs.append(orange_tube("tr1_" + sg, [A, HB], col))
        objs.append(orange_tube("tr2_" + sg, [B, HB], col))
        objs.append(orange_tube("tr3_" + sg, [B, RB], col))
        objs.append(orange_tube("tr4_" + sg, [Cc, RB], col))
        objs.append(orange_tube("tr5_" + sg, [HB, RB], col, r=0.009))
        for nm, p in (("A", A), ("B", B), ("C", Cc)):
            objs += lug("lug%s_%s" % (nm, sg), p, col, "Y", 0.0165, 0.032)
        for nm, p in (("HB", HB), ("RB", RB)):
            objs += lug("lug%s_%s" % (nm, sg), p, col, "Y", 0.0185, 0.024, both=False if s > 0 else False)
        # head tower plate
        tower = [(0.185, 0.757), (0.295, 0.899), (0.336, 0.890), (0.330, 0.800), (0.300, 0.722), (0.262, 0.668),
                 (0.226, 0.668), (0.192, 0.705)]
        objs.append(plate_xz("tower_" + sg, tower, s * 0.127, 0.012, col,
                             holes=[(0.262, 0.775, 0.026), (0.232, 0.715, 0.012), (0.292, 0.842, 0.013)]))
        # guide block for the seat carriage
        g = geo.box("guide_" + sg, 0.044, 0.040, 0.044, bevel=0.003, col=col, mat=M("alu_machined"),
                    loc=(-0.315, s * CARR_Y, CARR_Z))
        ped = geo.box("guideped_" + sg, 0.030, 0.050, 0.014, bevel=0.002, col=col, mat=M("ano_orange"),
                      loc=(-0.315, s * 0.112, 0.7515))
        objs += [g, ped]
        for bx in (-0.015, 0.015):
            b = geo.bolt("guide_b", 0.005, col=col, mat=M("steel_dark"), washer=False)
            b.location = (-0.315 + bx * 0.9, s * CARR_Y, CARR_Z + 0.022)
            objs.append(b)
        # shock / strut outrigger and foot peg hanger
        hang = [(-0.215, 0.545), (-0.165, 0.545), (-0.150, 0.500), (-0.205, 0.375), (-0.250, 0.350), (-0.262, 0.385),
                (-0.228, 0.470)]
        objs.append(plate_xz("hanger_" + sg, hang, s * 0.214, 0.010, col, "alu_brushed",
                             holes=[(-0.196, 0.468, 0.014), (-0.222, 0.404, 0.008)]))
        peg = geo.cyl("peg_" + sg, 0.0125, 0.085, 32, bevel=0.002, axis="Y", col=col, mat=M("alu_machined"),
                      loc=(-0.238, s * 0.263, 0.374))
        objs.append(peg)
        # knurl rings on the peg
        for k in range(6):
            rg = geo.torus("pegring", 0.0125, 0.0016, 32, 8, "Y", col, M("ano_black"),
                           loc=(-0.238, s * (0.232 + 0.011 * k), 0.374))
            objs.append(rg)
        sp = geo.cyl("pegspacer_" + sg, 0.009, 0.012, 24, axis="Y", col=col, mat=M("steel_polished"),
                     loc=(-0.190, s * 0.203, 0.520))
        objs.append(sp)
    # cross tubes
    objs.append(orange_tube("crossA", [(0.250, -0.118, 0.745), (0.250, 0.118, 0.745)], col))
    objs.append(orange_tube("crossB", [(-0.02, -0.118, 0.745), (-0.02, 0.118, 0.745)], col, r=0.009))
    objs.append(orange_tube("crossC", [(-0.33, -0.125, 0.745), (-0.33, 0.125, 0.745)], col))
    objs.append(geo.cyl("towercross", 0.011, 0.262, 24, axis="Y", col=col, mat=M("ano_orange"), loc=(0.232, 0, 0.715)))
    # left pivot plate (single sided swingarm) and its inner bracket
    piv = [(-0.322, 0.345), (-0.215, 0.335), (-0.195, 0.500), (-0.235, 0.585), (-0.300, 0.585), (-0.322, 0.470)]
    objs.append(plate_xz("pivot_plate", piv, 0.192, 0.012, col,
                         holes=[(-0.255, 0.505, 0.022), (-0.268, 0.553, 0.008)]))
    objs.append(orange_tube("piv_tube", [(-0.33, 0.125, 0.745), (-0.268, 0.186, 0.585)], col))
    inner = [(-0.322, 0.360), (-0.250, 0.360), (-0.250, 0.450), (-0.322, 0.450)]
    objs.append(plate_xz("pivot_inner", inner, 0.060, 0.010, col, "alu_machined"))
    cap = geo.cyl("pivot_cap", 0.026, 0.012, 48, bevel=0.003, axis="Y", col=col, mat=M("alu_turned"),
                  loc=(C.PIVOT[0], 0.204, C.PIVOT[2]))
    cap.rotation_euler = (0, 0, 0)
    objs.append(cap)
    b = geo.bolt("pivot_capb", 0.012, col=col, mat=M("steel_polished"), washer=False)
    b.rotation_euler = (-math.pi / 2, 0, 0)
    b.location = (C.PIVOT[0], 0.209, C.PIVOT[2])
    objs.append(b)
    # shock top eye bracket on the left rail
    br = [(-0.090, 0.752), (-0.020, 0.752), (-0.030, 0.690), (-0.070, 0.684)]
    objs.append(plate_xz("shock_bracket", br, 0.199, 0.008, col, "ano_orange"))
    objs.append(orange_tube("shock_out", [(-0.02, 0.118, 0.745), (-0.04, 0.199, 0.735)], col, r=0.008))
    # fuel cell between the rails
    cell = geo.box("fuel_cell", 0.44, 0.176, 0.052, bevel=0.006, segs=3, col=col, mat=M("ano_black"),
                   loc=(-0.03, 0, 0.732))
    objs.append(cell)
    for sx in (-0.19, 0.13):
        strap = geo.box("cell_strap", 0.022, 0.182, 0.056, bevel=0.002, col=col, mat=M("alu_brushed"), loc=(sx, 0, 0.732))
        objs.append(strap)
    return objs


# ----------------------------------------------------------------------------- head carrier (F-local)
def build_head(col):
    """Head tube + lugs in fork-frame coordinates (attach to the HEAD node with F as world matrix)."""
    objs = []
    ht = geo.lathe("head_tube", [(0.0175, -0.172), (0.031, -0.172), (0.034, -0.166), (0.034, -0.152), (0.030, -0.146),
                                 (0.030, -0.060), (0.034, -0.054), (0.034, -0.040), (0.031, -0.034), (0.0175, -0.034)],
                   64, "Z", col, M("ano_orange_turned"), closed=True)
    objs.append(ht)
    hx, hy, hz = C.HINGE_C_F
    up = geo.prism("head_lug_up", [(-0.020, -0.075), (-0.020, -0.038), (hx - 0.004, hz + 0.022), (hx - 0.022, hz + 0.008),
                                   (hx - 0.022, hz - 0.014), (hx - 0.006, hz - 0.026)], 0.118, bevel=0.002,
                   plane="XZ", col=col, mat=M("ano_orange"))
    objs.append(up)
    pin = geo.cyl("hinge_pin", 0.0085, 0.292, 32, bevel=0.001, axis="Y", col=col, mat=M("steel_polished"), loc=(hx, 0, hz))
    objs.append(pin)
    for s in (1, -1):
        c = geo.cyl("hinge_cap", 0.015, 0.006, 32, bevel=0.0015, axis="Y", col=col, mat=M("alu_machined"),
                    loc=(hx, s * 0.140, hz))
        objs.append(c)
    lo = geo.prism("head_lug_lo", [(-0.020, -0.165), (-0.020, -0.128), (-0.062, -0.132), (-0.074, -0.150), (-0.062, -0.168)],
                   0.150, bevel=0.002, plane="XZ", col=col, mat=M("ano_orange"))
    objs.append(lo)
    return objs


STRUT_HEAD_F = (-0.060, 0.066, -0.150)      # strut eye on the head carrier (fork frame), +-y
STRUT_FRAME = (0.232, 0.066, 0.715)         # strut eye on the frame tower cross tube (world), +-y


# ----------------------------------------------------------------------------- arm links
def _stadium(length, r, n=14, x0=0.0):
    pts = []
    for k in range(n + 1):
        a = math.pi / 2 + math.pi * k / n
        pts.append((x0 + r * math.cos(a), r * math.sin(a)))
    for k in range(n + 1):
        a = -math.pi / 2 + math.pi * k / n
        pts.append((x0 + length + r * math.cos(a), r * math.sin(a)))
    return pts


def arm_link(name, length, col, w=0.024, th=0.008, mat="ano_orange", pocket=True):
    """Machined link along local +X from pivot (0,0,0) to (length,0,0); thickness along local Y.
    Lightening slot, bearing bosses, pins with circlips, a row of small bolts."""
    r = w / 2.0
    o = geo.prism(name, _stadium(length, r), th, bevel=0.0012, plane="XZ", col=col, mat=M(mat))
    if pocket and length > 0.11:
        cut = geo.prism(name + "_cut", _stadium(length - 4.4 * r, r * 0.36, x0=2.2 * r), th * 3, plane="XZ", col=col)
        m = o.modifiers.new("slot", "BOOLEAN")
        m.operation = "DIFFERENCE"
        m.object = cut
        try:
            m.solver = "EXACT"
        except Exception:
            pass
        geo.apply_mods(o)
        bpy.data.objects.remove(cut, do_unlink=True)
        o.data.shade_flat()
    parts = [o]
    for x in (0.0, length):
        boss = geo.cyl(name + "_boss", r * 0.80, th + 0.006, 32, bevel=0.001, axis="Y", col=col, mat=M("alu_turned"),
                       loc=(x, 0, 0))
        parts.append(boss)
        pinh = geo.cyl(name + "_pin", r * 0.36, th + 0.011, 24, bevel=0.0008, axis="Y", col=col,
                       mat=M("steel_polished"), loc=(x, 0, 0))
        parts.append(pinh)
        for sy in (1, -1):
            clip = geo.torus(name + "_clip", r * 0.46, 0.0007, 24, 6, "Y", col, M("steel_dark"),
                             loc=(x, sy * (th / 2 + 0.0036), 0))
            parts.append(clip)
    # small cap screws flanking the slot
    for x in (1.55 * r, length - 1.55 * r):
        for sy in (1, -1):
            bb = geo.bolt(name + "_b", 0.0034, col=col, mat=M("steel_dark"), washer=False)
            bb.rotation_euler = (-math.pi / 2 * sy, 0, 0)
            bb.location = (x, sy * th / 2, 0)
            parts.append(bb)
    return geo.join(parts, name)


def arm_strut(name, length, col):
    """Small hydraulic strut for an arm: returns (body, rod), both along local +X from their own eye.
    Body length 0.60 * length, rod length 0.55 * length."""
    lb, lr = 0.60 * length, 0.55 * length
    b = []
    b.append(geo.cyl(name + "_be", 0.0075, 0.008, 20, bevel=0.001, axis="Y", col=col, mat=M("alu_machined"), loc=(0, 0, 0)))
    b.append(geo.cyl(name + "_bt", 0.0064, lb - 0.010, 24, axis="X", col=col, mat=M("ano_black"), loc=(0.010 + (lb - 0.010) / 2, 0, 0)))
    b.append(geo.cyl(name + "_bg", 0.0076, 0.008, 24, bevel=0.001, axis="X", col=col, mat=M("alu_turned"), loc=(lb - 0.004, 0, 0)))
    b.append(geo.cyl(name + "_bc", 0.0072, 0.006, 24, bevel=0.001, axis="X", col=col, mat=M("ano_orange_turned"), loc=(0.016, 0, 0)))
    b.append(geo.cyl(name + "_bp", 0.0026, 0.010, 12, axis="Z", col=col, mat=M("brass"), loc=(0.024, 0, 0.008)))
    body = geo.join(b, name + "_body")
    r = []
    r.append(geo.cyl(name + "_re", 0.0068, 0.007, 20, bevel=0.001, axis="Y", col=col, mat=M("alu_machined"), loc=(0, 0, 0)))
    r.append(geo.cyl(name + "_rr", 0.0031, lr - 0.006, 16, axis="X", col=col, mat=M("chrome"), loc=(0.006 + (lr - 0.006) / 2, 0, 0)))
    r.append(geo.cyl(name + "_rn", 0.0048, 0.006, 6, axis="X", col=col, mat=M("steel_dark"), loc=(0.010, 0, 0)))
    rod = geo.join(r, name + "_rod")
    return body, rod


# ----------------------------------------------------------------------------- swingarm (local frames)
def build_sa_block(col):
    """World/final builder: universal block on the pivot + outboard bell crank for the shock."""
    objs = []
    px, py, pz = B_HINGE
    blk = geo.box("sa_block", 0.070, 0.086, 0.064, bevel=0.005, segs=3, col=col, mat=M("alu_machined"), loc=(px, py, pz))
    objs.append(blk)
    sh = geo.cyl("sa_shaft", 0.013, 0.20, 32, axis="Y", col=col, mat=M("steel_polished"), loc=(px, 0.150, pz))
    objs.append(sh)
    # bell crank outside the pivot plate
    crank = [(px - 0.024, pz - 0.022), (px + 0.024, pz - 0.022), (SHOCK_BOT[0] + 0.020, SHOCK_BOT[2] + 0.006),
             (SHOCK_BOT[0] + 0.006, SHOCK_BOT[2] + 0.020), (SHOCK_BOT[0] - 0.016, SHOCK_BOT[2] + 0.010),
             (px - 0.030, pz + 0.020)]
    objs.append(plate_xz("sa_crank", crank, 0.2265, 0.009, col, "ano_orange",
                         holes=[(px - 0.004, pz + 0.050, 0.010)]))
    return objs


def build_link1(col):
    """Local: hinge at origin, beam along -X to (-LINK1, 0, 0). Twin rails with a slot between."""
    L = C.LINK1
    objs = []
    for sz in (1, -1):
        rail = geo.box("l1_rail", L + 0.052, 0.050, 0.019, bevel=0.004, segs=3, col=col, mat=M("alu_brushed"),
                       loc=(-L / 2, 0, sz * 0.0415))
        objs.append(rail)
    web = geo.box("l1_web", L - 0.11, 0.005, 0.066, bevel=0.0012, col=col, mat=M("alu_plate"), loc=(-L / 2, -0.0225, 0))
    objs.append(web)
    for x in (0.0, -L):
        pin = geo.cyl("l1_pin", 0.0105, 0.118, 32, bevel=0.0015, col=col, mat=M("steel_polished"), loc=(x, 0, 0))
        objs.append(pin)
        for sz in (1, -1):
            cp = geo.cyl("l1_cap", 0.018, 0.006, 32, bevel=0.002, col=col, mat=M("ano_orange_turned"), loc=(x, 0, sz * 0.054))
            objs.append(cp)
    for i in range(4):
        b = geo.bolt("l1_b", 0.005, col=col, mat=M("steel_dark"), washer=False)
        b.rotation_euler = (math.pi / 2, 0, 0)
        b.location = (-0.085 - i * 0.048, -0.025, 0.022 if i % 2 == 0 else -0.022)
        objs.append(b)
    return objs


def build_link2(col):
    """Local: elbow at origin, tongue along -X to the final drive flange at x = -(LINK2 - 0.075)."""
    xe = -(C.LINK2 - 0.075)
    objs = []
    tongue = geo.box("l2_tongue", abs(xe) + 0.040, 0.044, 0.060, bevel=0.005, segs=3, col=col, mat=M("alu_brushed"),
                     loc=(xe / 2 + 0.006, 0, 0))
    objs.append(tongue)
    eye = geo.cyl("l2_eye", 0.026, 0.060, 40, bevel=0.003, col=col, mat=M("alu_brushed"), loc=(0, 0, 0))
    objs.append(eye)
    fl = geo.cyl("l2_flange", 0.041, 0.010, 48, bevel=0.002, axis="X", col=col, mat=M("alu_machined"), loc=(xe + 0.005, 0, 0))
    objs.append(fl)
    for i in range(6):
        a = 2 * math.pi * i / 6 + 0.3
        b = geo.bolt("l2_b", 0.005, col=col, mat=M("steel_dark"), washer=False)
        b.rotation_euler = (0, math.pi / 2, 0)
        b.location = (xe + 0.010, 0.031 * math.cos(a), 0.031 * math.sin(a))
        objs.append(b)
    return objs


# ----------------------------------------------------------------------------- shock (local, along +X)
SHOCK_L0 = math.dist(SHOCK_TOP, SHOCK_BOT)


def build_shock(col):
    """Returns (top_part, bottom_part, spring, L0).  All along local +X.
    top_part: origin at the frame eye (body + reservoir), bottom_part: origin at the crank eye, pointing
    back to the body along +X (rod), spring: origin at its top seat, natural length returned."""
    L0 = SHOCK_L0
    body = []
    body.append(geo.cyl("sh_eye_t", 0.014, 0.022, 32, bevel=0.002, axis="Y", col=col, mat=M("alu_machined"), loc=(0, 0, 0)))
    body.append(geo.cyl("sh_body", 0.021, 0.118, 48, bevel=0.002, axis="X", col=col, mat=M("ano_black"), loc=(0.082, 0, 0)))
    body.append(geo.cyl("sh_collar", 0.034, 0.010, 48, bevel=0.002, axis="X", col=col, mat=M("ano_orange_turned"), loc=(0.040, 0, 0)))
    body.append(geo.cyl("sh_collar2", 0.030, 0.006, 48, bevel=0.0015, axis="X", col=col, mat=M("alu_turned"), loc=(0.030, 0, 0)))
    res = geo.cyl("sh_res", 0.015, 0.075, 32, bevel=0.003, axis="X", col=col, mat=M("alu_turned"), loc=(0.060, 0, 0.040))
    body.append(res)
    body.append(geo.box("sh_resb", 0.022, 0.016, 0.030, bevel=0.002, col=col, mat=M("alu_machined"), loc=(0.030, 0, 0.024)))
    top = geo.join(body, "shock_top")
    bot = []
    bot.append(geo.cyl("sh_eye_b", 0.014, 0.022, 32, bevel=0.002, axis="Y", col=col, mat=M("alu_machined"), loc=(0, 0, 0)))
    bot.append(geo.cyl("sh_rod", 0.0075, 0.20, 32, axis="X", col=col, mat=M("chrome"), loc=(0.105, 0, 0)))
    bot.append(geo.cyl("sh_seat", 0.033, 0.008, 48, bevel=0.002, axis="X", col=col, mat=M("alu_turned"), loc=(0.028, 0, 0)))
    bot.append(geo.cyl("sh_bump", 0.014, 0.022, 32, bevel=0.003, axis="X", col=col, mat=M("rubber"), loc=(0.044, 0, 0)))
    bottom = geo.join(bot, "shock_bot")
    # spring between the collar (x = 0.045 from top eye) and the seat (0.032 from bottom eye)
    Ls = L0 - 0.045 - 0.032
    turns = 7.0
    pts = []
    n = int(turns * 28)
    for i in range(n + 1):
        u = i / n
        a = 2 * math.pi * turns * u
        # closed, ground ends: pitch goes to zero in the first / last coil
        e = 0.5 - 0.5 * math.cos(math.pi * min(1.0, u * turns)) if u * turns < 1 else 1.0
        x = Ls * (u - 0.5 / turns * (math.sin(2 * math.pi * min(u * turns, 0.5) * 0) + 0))
        pts.append((Ls * u, 0.0275 * math.cos(a), 0.0275 * math.sin(a)))
    spring = geo.tube("shock_spring", pts, 0.0048, 12, col=col, mat=M("ano_orange"))
    return top, bottom, spring, L0, Ls


# ----------------------------------------------------------------------------- seat carriage (world, final)
def build_seat(col):
    x0, x1 = SEAT_X
    nx, ny = 56, 18
    zb = C.SEAT_Z[0] + 0.004
    zt = C.SEAT_Z[1]
    bm = bmesh.new()

    def hw(x):
        u = (x - x0) / (x1 - x0)
        return 0.104 + 0.022 * u

    def top(x, v):
        u = (x - x0) / (x1 - x0)
        z = zt - 0.020 * abs(v) ** 3.2
        # rear hump (bum stop)
        z += 0.050 * max(0.0, 1.0 - u / 0.22) ** 1.6
        # front nose roll-off
        z -= 0.018 * max(0.0, (u - 0.93) / 0.07) ** 2
        # tuck-and-roll grooves
        if 0.24 < u < 0.92:
            d = ((u - 0.24) * (x1 - x0)) % 0.036
            d = min(d, 0.036 - d)
            z -= 0.0042 * math.exp(-(d / 0.0038) ** 2) * (1 - abs(v) ** 6)
        return z

    grid = []
    for i in range(nx + 1):
        x = x0 + (x1 - x0) * i / nx
        row = []
        for j in range(ny + 1):
            v = -1 + 2 * j / ny
            row.append(bm.verts.new((x, v * hw(x), top(x, v))))
        grid.append(row)
    for i in range(nx):
        for j in range(ny):
            bm.faces.new((grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]))
    # skirt down to the pan
    border = [grid[i][0] for i in range(nx + 1)] + [grid[nx][j] for j in range(1, ny + 1)] + \
             [grid[i][ny] for i in range(nx - 1, -1, -1)] + [grid[0][j] for j in range(ny - 1, 0, -1)]
    low = []
    for v in border:
        low.append(bm.verts.new((v.co.x * 1.0 + (0.004 if v.co.x < (x0 + x1) / 2 else -0.004) * 0, v.co.y * 0.93, zb)))
    nb = len(border)
    for k in range(nb):
        a, b = border[k], border[(k + 1) % nb]
        c, d = low[(k + 1) % nb], low[k]
        bm.faces.new((a, d, c, b))
    bm.faces.new(low)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    seat = geo.obj_from_bm("seat", bm, col, smooth=60)
    geo.set_mat(seat, M("leather"))
    geo.subsurf_mod(seat, 1)
    objs = [seat]
    # stitch lines along both shoulders
    for s in (1, -1):
        pts = []
        for i in range(8, nx - 2):
            x = x0 + (x1 - x0) * i / nx
            v = s * 0.78
            pts.append((x, v * hw(x), top(x, v) + 0.0006))
        st = geo.tube("stitch", pts, 0.0011, 8, col=col, mat=M("stitch"))
        objs.append(st)
    pan = geo.box("seat_pan", (x1 - x0) - 0.02, 0.19, 0.006, bevel=0.002, col=col, mat=M("alu_brushed"),
                  loc=((x0 + x1) / 2, 0, C.SEAT_Z[0] + 0.001))
    objs.append(pan)
    return objs


def build_carriage(col):
    objs = []
    xa, xb = CARR_X
    for s in (1, -1):
        r = geo.cyl("carr_rail", 0.011, xb - xa, 32, bevel=0.002, axis="X", col=col, mat=M("ano_orange"),
                    loc=((xa + xb) / 2, s * CARR_Y, CARR_Z))
        objs.append(r)
        # outrigger lug for the folding strut
        o = geo.box("carr_out", 0.034, 0.066, 0.020, bevel=0.003, col=col, mat=M("ano_orange"), loc=(-0.62, s * 0.126, CARR_Z))
        objs.append(o)
        # seat pan stand-offs
        for x in (-0.70, -0.52, -0.36):
            so = geo.cyl("carr_so", 0.008, 0.012, 20, col=col, mat=M("alu_machined"), loc=(x, s * 0.085, CARR_Z + 0.013))
            objs.append(so)
    for x in (-1.045, -0.86, -0.66):
        cb = geo.cyl("carr_cross", 0.008, 2 * CARR_Y, 24, axis="Y", col=col, mat=M("ano_orange"), loc=(x, 0, CARR_Z))
        objs.append(cb)
    # tail light unit at the rear end
    hous = geo.box("tail_housing", 0.030, 0.230, 0.034, bevel=0.004, segs=3, col=col, mat=M("ano_black"), loc=(-1.062, 0, 0.812))
    objs.append(hous)
    for s in (1, -1):
        st = geo.box("tail_stay", 0.012, 0.012, 0.040, bevel=0.002, col=col, mat=M("ano_black"), loc=(-1.056, s * 0.085, 0.790))
        objs.append(st)
    led = geo.box("taillight_emit", 0.006, 0.205, 0.014, bevel=0.002, col=col, mat=M("emit_red"), loc=(-1.0785, 0, 0.812))
    mats.set_emit(led, 0.0)
    lens = geo.box("tail_lens", 0.004, 0.214, 0.020, bevel=0.0015, col=col, mat=M("glass"), loc=(-1.0825, 0, 0.812))
    objs += build_seat(col)
    return objs, led, lens


CARR_STRUT_FRAME = (-0.262, 0.214, 0.555)     # on the hanger / pivot plates, +-y
CARR_STRUT_CARR = (-0.62, 0.150, CARR_Z)      # outrigger on the carriage (final), +-y


# ----------------------------------------------------------------------------- stand feet (local)
def build_stand_leg(col):
    """Local: hinge at origin, leg pointing down -Z, length 0.125, with a foot pad."""
    leg = geo.box("stand_leg", 0.018, 0.016, 0.115, bevel=0.003, col=col, mat=M("ano_orange"), loc=(0, 0, -0.058))
    pad = geo.cyl("stand_pad", 0.020, 0.010, 32, bevel=0.003, col=col, mat=M("rubber"), loc=(0, 0, -0.120))
    eye = geo.cyl("stand_eye", 0.012, 0.022, 24, bevel=0.002, axis="Y", col=col, mat=M("alu_machined"), loc=(0, 0, 0))
    return geo.join([leg, pad, eye], "stand_leg")
