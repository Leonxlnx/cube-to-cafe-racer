"""Front end of the cafe racer: USD fork, billet triple clamps, clip-ons with controls, round headlight,
twin analogue gauges, radial four-piston calipers and a short rolled-edge aluminium fender.

build(col) -> dict of root empties, all with fork-frame (F) orientation (see common.py):
    "steer"     origin F_O                        clamps, stem, outer tubes, headlight, gauges
    "lower"     origin F_O                        stanchions, feet, axle, calipers, fender (telescopes along F.z)
    "clipon_L"  origin F(0.032, +0.105, -0.055)   left clip-on (rotate about local Z = fork tube axis)
    "clipon_R"  origin F(0.032, -0.105, -0.055)   right clip-on

Everything is modelled in F coordinates (or in the clip-on root frame) and parented to the roots with an
identity parent inverse, so object local space == root space.
Animatable extras: "headlight_emit" (custom prop "emit"), "gauge_needle_L/R" (rotate about local Z,
0 -> -270 deg sweeps zero -> full scale), "clipon_L_lever"/"clipon_R_lever" (pivot = object origin, local Z).
"""
import math
import bpy
import bmesh
from mathutils import Vector, Matrix

import common as C
import geo
import mats

PI = math.pi
RAKE = C.RAKE
TX = C.FORK_OFFSET
TY = C.FORK_HALF_SPAN
WC = Vector((TX, 0.0, -C.FORK_LEN))          # wheel centre in F
CAL_PHI = math.radians(150.0)                 # caliper centre direction around the axle (F x->z plane)
HEAD_C = Vector((0.215, 0.0, -0.105))         # headlight centre in F
GAUGE_TILT = math.radians(15.0)

_STATE = {"col": None}


def _col():
    return _STATE["col"]


def rad(a):
    return math.radians(a)


# ============================================================================ matrices / mesh utils
def frame(origin=(0, 0, 0), z=(0, 0, 1), x_hint=(1, 0, 0)):
    """Proper rotation frame with local Z along z, local X as close as possible to x_hint."""
    z = Vector(z).normalized()
    xh = Vector(x_hint)
    x = xh - z * xh.dot(z)
    if x.length < 1e-6:
        xh = Vector((0, 1, 0)) if abs(z.y) < 0.9 else Vector((1, 0, 0))
        x = xh - z * xh.dot(z)
    x.normalize()
    y = z.cross(x)
    M = Matrix((x, y, z)).transposed().to_4x4()
    M.translation = Vector(origin)
    return M


def basis(origin, X, Y, Z):
    M = Matrix((Vector(X), Vector(Y), Vector(Z))).transposed().to_4x4()
    M.translation = Vector(origin)
    return M


def T(x, y=None, z=None):
    if y is None:
        return Matrix.Translation(Vector(x))
    return Matrix.Translation(Vector((x, y, z)))


# local (X, Y, Z) -> F (X, Z, Y): polygon drawn in F's XZ plane, extruded along F.y (reflection)
XZ = Matrix(((1, 0, 0, 0), (0, 0, 1, 0), (0, 1, 0, 0), (0, 0, 0, 1)))


def xform(ob, M):
    """Transform mesh data by M (fixing the winding for reflections)."""
    if M is None:
        return ob
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bmesh.ops.transform(bm, matrix=M, verts=bm.verts)
    if M.to_3x3().determinant() < 0:
        bmesh.ops.reverse_faces(bm, faces=bm.faces[:])
    bm.normal_update()
    bm.to_mesh(ob.data)
    bm.free()
    ob.data.update()
    return ob


def setmat(ob, mat):
    if mat:
        geo.set_mat(ob, mats.get(mat))
    return ob


def remove(ob):
    me = ob.data
    bpy.data.objects.remove(ob, do_unlink=True)
    if me is not None and me.users == 0:
        bpy.data.meshes.remove(me)


def mk(name, bm, mat=None, smooth=40.0):
    ob = geo.obj_from_bm(name, bm, _col(), smooth=smooth)
    return setmat(ob, mat)


# ============================================================================ primitives (object space = current space)
def box(name, a, b, M=None, bevel=0.0, segs=2, mat=None):
    mn = Vector([min(a[i], b[i]) for i in range(3)])
    mx = Vector([max(a[i], b[i]) for i in range(3)])
    s = mx - mn
    c = (mn + mx) / 2
    ob = geo.box(name, s.x, s.y, s.z, bevel=bevel, segs=segs, col=_col(), loc=tuple(c))
    setmat(ob, mat)
    return xform(ob, M)


def cylz(name, r, z0, z1, M=None, segs=48, bevel=0.0, r2=None, mat=None):
    if z1 < z0:
        z0, z1 = z1, z0
    ob = geo.cyl(name, r, z1 - z0, segs=segs, bevel=bevel, col=_col(), loc=(0, 0, (z0 + z1) / 2), r2=r2)
    setmat(ob, mat)
    return xform(ob, M)


def lathe(name, prof, M=None, segs=64, closed=False, mat=None, smooth=40.0):
    ob = geo.lathe(name, prof, segs, "Z", _col(), None, closed=closed, smooth=smooth)
    setmat(ob, mat)
    return xform(ob, M)


def _dedup(pts, eps=1e-7):
    out = []
    for p in pts:
        if not out or (abs(out[-1][0] - p[0]) > eps or abs(out[-1][1] - p[1]) > eps):
            out.append((p[0], p[1]))
    if len(out) > 2 and abs(out[0][0] - out[-1][0]) < eps and abs(out[0][1] - out[-1][1]) < eps:
        out.pop()
    return out


def prism(name, pts, z0, z1, M=None, mat=None):
    """Polygon (local XY) extruded from z0 to z1 (closed solid)."""
    pts = _dedup(pts)
    n = len(pts)
    area = sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1] for i in range(n))
    if area < 0:
        pts = pts[::-1]
    if z1 < z0:
        z0, z1 = z1, z0
    bm = bmesh.new()
    b = [bm.verts.new((x, y, z0)) for x, y in pts]
    t = [bm.verts.new((x, y, z1)) for x, y in pts]
    bm.faces.new(b[::-1])
    bm.faces.new(t)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((b[i], b[j], t[j], t[i]))
    ob = mk(name, bm, mat, smooth=None)
    return xform(ob, M)


def bolt(name, d, pos, direction, kind="socket", washer=False, head_h=None, mat="steel_dark", x_hint=(1, 0, 0)):
    ob = geo.bolt(name, d=d, head_h=head_h, kind=kind, col=_col(), washer=washer)
    setmat(ob, mat)
    return xform(ob, frame(pos, direction, x_hint))


def sphere(name, r, M=None, segs=32, rings=16, mat=None):
    prof = [(r * math.sin(PI * k / rings), -r * math.cos(PI * k / rings)) for k in range(rings + 1)]
    return lathe(name, prof, M, segs, mat=mat, smooth=80.0)


# ============================================================================ 2D helpers
def circ(cx, cy, r, n=48, a0=0.0):
    return [(cx + r * math.cos(a0 + 2 * PI * i / n), cy + r * math.sin(a0 + 2 * PI * i / n)) for i in range(n)]


def hull2d(points):
    pts = sorted(set((round(p[0], 8), round(p[1], 8)) for p in points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def rrect(cx, cy, hx, hy, r, n=8):
    r = min(r, hx, hy)
    pts = []
    for (qx, qy, a0) in ((hx - r, hy - r, 0), (-hx + r, hy - r, 90), (-hx + r, -hy + r, 180), (hx - r, -hy + r, 270)):
        for k in range(n + 1):
            a = rad(a0 + 90.0 * k / n)
            pts.append((cx + qx + r * math.cos(a), cy + qy + r * math.sin(a)))
    return _dedup(pts)


def stadium(p1, p2, r, n=12):
    p1, p2 = Vector(p1), Vector(p2)
    d = (p2 - p1).normalized()
    a = math.atan2(d.y, d.x)
    pts = []
    for k in range(n + 1):
        t = a - PI / 2 + PI * k / n
        pts.append((p2.x + r * math.cos(t), p2.y + r * math.sin(t)))
    for k in range(n + 1):
        t = a + PI / 2 + PI * k / n
        pts.append((p1.x + r * math.cos(t), p1.y + r * math.sin(t)))
    return pts


def fillet_poly(pts, r, n=6, min_turn=8.0):
    P = [Vector((p[0], p[1])) for p in _dedup(pts)]
    N = len(P)
    out = []
    for i in range(N):
        a, b_, c = P[i - 1], P[i], P[(i + 1) % N]
        d1, d2 = a - b_, c - b_
        l1, l2 = d1.length, d2.length
        if l1 < 1e-9 or l2 < 1e-9:
            out.append(b_)
            continue
        d1.normalize()
        d2.normalize()
        ang = d1.angle(d2)
        if ang > PI - rad(min_turn) or r <= 0:
            out.append(b_)
            continue
        t = min(r / math.tan(ang / 2), l1 * 0.45, l2 * 0.45)
        rr = t * math.tan(ang / 2)
        p1, p2 = b_ + d1 * t, b_ + d2 * t
        cen = b_ + (d1 + d2).normalized() * (rr / math.sin(ang / 2))
        a1 = math.atan2(p1.y - cen.y, p1.x - cen.x)
        a2 = math.atan2(p2.y - cen.y, p2.x - cen.x)
        da = a2 - a1
        while da > PI:
            da -= 2 * PI
        while da < -PI:
            da += 2 * PI
        for k in range(n + 1):
            aa = a1 + da * k / n
            out.append(cen + Vector((math.cos(aa), math.sin(aa))) * rr)
    return [(p.x, p.y) for p in out]


def catmull(pts, sub=6):
    P = [Vector(p) for p in pts]
    out = []
    for i in range(len(P) - 1):
        p0 = P[i - 1] if i > 0 else P[i] * 2 - P[i + 1]
        p1, p2 = P[i], P[i + 1]
        p3 = P[i + 2] if i + 2 < len(P) else P[i + 1] * 2 - P[i]
        for k in range(sub):
            t = k / sub
            t2, t3 = t * t, t * t * t
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
                              (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
    out.append(P[-1])
    return out


# ============================================================================ booleans / joins
def _apply_mods(ob):
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    old = ob.data
    ob.modifiers.clear()
    ob.data = me
    if old.users == 0:
        bpy.data.meshes.remove(old)
    return ob


def boolean(target, others, op="DIFFERENCE"):
    if not isinstance(others, (list, tuple)):
        others = [others]
    others = [o for o in others if o is not None]
    if not others:
        return target
    if op == "DIFFERENCE" and len(others) > 1:
        cutter = geo.join(others, "cutter")
        others = [cutter]
    keep = [m for m in target.data.materials if m is not None][:1]
    for o in others:
        m = target.modifiers.new("bool", "BOOLEAN")
        m.operation = op
        m.solver = "EXACT"
        m.object = o
        try:
            m.use_self = True
        except Exception:
            pass
        _apply_mods(target)
    for o in others:
        remove(o)
    me = target.data
    me.materials.clear()
    for m in keep:
        me.materials.append(m)
    for p in me.polygons:
        p.material_index = 0
    return target


def union(target, others):
    return boolean(target, others, "UNION")


def cut(target, others):
    return boolean(target, others, "DIFFERENCE")


def join(objs, name, M=None):
    objs = [o for o in objs if o is not None]
    ob = geo.join(objs, name)
    ob.data.name = name
    if M is not None:
        ob.data.transform(M.inverted())
        ob.data.update()
        ob.matrix_basis = M
    return ob


def finish(ob, smooth=35.0, bevel=0.0, segs=3, angle=30.0):
    geo.set_smooth(ob, smooth)
    if bevel > 0:
        m = geo.bevel_mod(ob, bevel, segs, angle, harden=True)
        try:
            m.use_clamp_overlap = True
            m.miter_outer = "MITER_ARC"
        except Exception:
            pass
    return ob


# ============================================================================ STEER: triple clamps
def _clamp_outline():
    pts = []
    for s in (1, -1):
        pts += circ(TX, s * TY, 0.040, 72)
        pts += circ(TX, s * 0.140, 0.016, 36)
    pts += circ(0.0, 0.0, 0.036, 72)
    return hull2d(pts)


def triple_clamp(name, z0, z1, pockets_top, stem_r, bolt_zs, bolts):
    body = prism(name, _clamp_outline(), z0, z1)
    e = 0.01
    cs = [cylz("c", 0.120, z0 - e, z1 + e, M=T(0.174, 0, 0), segs=160)]
    for s in (1, -1):
        cs.append(cylz("c", 0.070, z0 - e, z1 + e, M=T(-0.080, s * 0.080, 0), segs=128))
        cs.append(cylz("c", 0.0272, z0 - e, z1 + e, M=T(TX, s * TY, 0), segs=72))
        cs.append(box("c", (TX - 0.0007, s * 0.120, z0 - e), (TX + 0.0007, s * 0.175, z1 + e)))
        for zb in bolt_zs:
            cs.append(cylz("c", 0.0028, -0.05, 0.05, M=frame((TX, s * 0.141, zb), (1, 0, 0)), segs=20))
            cs.append(cylz("c", 0.0054, 0.0, 0.05, M=frame((TX + 0.0125, s * 0.141, zb), (1, 0, 0)), segs=28))
            bolts.append(bolt("b", 0.006, (TX + 0.0125, s * 0.141, zb), (1, 0, 0)))
        pk = rrect(0.019, s * 0.049, 0.028, 0.0155, 0.008)
        dpt = 0.014
        if pockets_top:
            cs.append(prism("c", pk, z1 - dpt, z1 + e))
        else:
            cs.append(prism("c", pk, z0 - e, z0 + dpt))
    cs.append(cylz("c", stem_r, z0 - e, z1 + e, segs=48))
    cut(body, cs)
    return body


def head_bracket(s, bolts, extras):
    """Machined headlight ear clamping the outer tube at s side."""
    a = Vector((math.cos(RAKE), 0, -math.sin(RAKE)))
    mp = HEAD_C + a * (-0.030)
    xb, zb = mp.x, mp.z
    zr0, zr1 = -0.125, -0.095
    zc = (zr0 + zr1) / 2
    ring = prism("hb", hull2d(circ(TX, s * TY, 0.035, 72) + circ(TX, s * 0.141, 0.0115, 36)), zr0, zr1)
    y0, y1 = sorted((s * 0.094, s * 0.102))
    arm_pts = fillet_poly([(0.048, zr0 + 0.0015), (xb, zb - 0.0105), (xb, zb + 0.0105), (0.048, zr1 - 0.0015)], 0.004)
    arm = prism("arm", arm_pts, y0, y1, M=XZ)
    boss = cylz("boss", 0.0115, y0, y1, M=T(xb, 0, zb) @ XZ, segs=40)
    union(ring, [arm, boss])

    def zmid(x):
        return zc + (zb - zc) * (x - 0.048) / (xb - 0.048)

    cs = [cylz("c", 0.0272, zr0 - 0.01, zr1 + 0.01, M=T(TX, s * TY, 0), segs=72),
          box("c", (TX - 0.0007, s * 0.118, zr0 - 0.01), (TX + 0.0007, s * 0.17, zr1 + 0.01)),
          cylz("c", 0.0026, -0.05, 0.05, M=frame((TX, s * 0.141, zc), (1, 0, 0)), segs=20),
          cylz("c", 0.0052, 0.0, 0.05, M=frame((TX + 0.0085, s * 0.141, zc), (1, 0, 0)), segs=28),
          prism("c", stadium((0.090, zmid(0.090)), (0.146, zmid(0.146)), 0.0036), -0.2, 0.2, M=XZ),
          cylz("c", 0.0032, -0.2, 0.2, M=T(xb, 0, zb) @ XZ, segs=20)]
    cut(ring, cs)
    bolts.append(bolt("b", 0.005, (TX + 0.0085, s * 0.141, zc), (1, 0, 0)))
    # spacer between shell and ear, mount bolt
    ys0, ys1 = sorted((s * 0.0880, s * 0.0942))
    extras.append(cylz("spacer", 0.0068, ys0, ys1, M=T(xb, 0, zb) @ XZ, segs=32, bevel=0.0004))
    bolts.append(bolt("b", 0.006, (xb, s * 0.102, zb), (0, s, 0), kind="socket", washer=True))
    return ring


def gauge_frames():
    n = Vector((-math.sin(GAUGE_TILT), 0, math.cos(GAUGE_TILT)))
    u = Vector((math.cos(GAUGE_TILT), 0, math.sin(GAUGE_TILT)))
    xr = Vector((0, -1, 0))
    out = []
    for s in (1, -1):
        c = Vector((0.100, s * 0.044, 0.048))
        out.append((s, basis(c, xr, u, n)))
    return out, n, u


def gauge_bracket():
    fr, n, u = gauge_frames()
    cmid = Vector((0.100, 0.0, 0.048))
    G = basis(cmid - n * 0.034, (0, -1, 0), u, n)
    plate = prism("gb", rrect(0.0, -0.004, 0.075, 0.016, 0.008), -0.002, 0.002, M=G)
    pc = cmid - n * 0.034
    pa = pc - u * 0.017 - n * 0.0017
    pb = pc - u * 0.008 + n * 0.0017
    strap_pts = fillet_poly([(0.029, 0.0), (0.062, 0.0), (pa.x, pa.z), (pb.x, pb.z),
                             (0.058, 0.0055), (0.029, 0.0055)], 0.002)
    strap = prism("gs", strap_pts, -0.016, 0.016, M=XZ)
    union(plate, [strap])
    return plate


# ============================================================================ STEER: lathe parts
def outer_tube_profile():
    p = [(0.0, 0.012), (0.0252, 0.012), (0.0270, 0.0102), (0.0270, -0.3700)]
    for zg in (-0.372, -0.386):
        p += [(0.0270, zg), (0.0264, zg - 0.0006), (0.0264, zg - 0.0024), (0.0270, zg - 0.003)]
    p += [(0.0270, -0.4205), (0.0285, -0.4235), (0.0285, -0.4380), (0.0274, -0.4400),
          (0.0238, -0.4400), (0.0228, -0.4330), (0.0, -0.4330)]
    return p


def fork_cap(name):
    base = lathe("capbase", [(0.0, 0.0), (0.0262, 0.0), (0.0268, 0.0006), (0.0268, 0.0052), (0.0256, 0.0066),
                        (0.0, 0.0066)], segs=64, mat="alu_turned")
    hexa = cylz("hex", 0.0205, 0.0062, 0.0142, segs=6, bevel=0.0009, mat="alu_turned")
    adj = cylz("adj", 0.0108, 0.0138, 0.0198, segs=6, bevel=0.0007, mat="ano_orange_turned")
    star = [((0.0079 if k % 2 == 0 else 0.0072) * math.cos(2 * PI * k / 64),
             (0.0079 if k % 2 == 0 else 0.0072) * math.sin(2 * PI * k / 64)) for k in range(64)]
    ring = prism("adjr", star, 0.0196, 0.0218, mat="alu_turned")
    detent = prism("det", [(0.0110 * math.cos(2 * PI * k / 24 + (0.06 if j else -0.06)),
                            0.0110 * math.sin(2 * PI * k / 24 + (0.06 if j else -0.06)))
                           for k in range(24) for j in (0, 1)], 0.0196, 0.0199)
    cut(adj, [cylz("c", 0.0012, 0.0170, 0.0210, M=Matrix.Rotation(2 * PI * k / 12, 4, "Z") @ T(0.0101, 0, 0),
                   segs=8) for k in range(12)])
    remove(detent)
    slot = box("slot", (-0.0050, -0.0006, 0.0208), (0.0050, 0.0006, 0.0222))
    cut(ring, [slot])
    ob = join([base, hexa, adj, ring], name)
    return ob


def stem_nut():
    prof = [(0.0, -0.2158), (0.0145, -0.2158), (0.0160, -0.2143), (0.0160, 0.0002), (0.0232, 0.0002),
            (0.0246, 0.0016), (0.0246, 0.0115), (0.0232, 0.0130), (0.0122, 0.0130), (0.0108, 0.0116),
            (0.0108, 0.0065), (0.0, 0.0065)]
    ob = lathe("fr_stem", prof, segs=64, mat="alu_turned")
    # four spanner notches in the nut
    cs = []
    for k in range(4):
        a = PI / 4 + k * PI / 2
        cs.append(box("c", (0.0215, -0.0022, 0.0055), (0.030, 0.0022, 0.020),
                      M=Matrix.Rotation(a, 4, "Z")))
    cut(ob, cs)
    return ob


def dust_seal(s):
    prof = [(0.0218, -0.4525), (0.0236, -0.4525), (0.0285, -0.4488), (0.0293, -0.4445), (0.0293, -0.4385),
            (0.0282, -0.4370), (0.0222, -0.4370), (0.0218, -0.4400)]
    return lathe("seal", prof, M=T(TX, s * TY, 0), segs=64, closed=True, mat="rubber")


# ============================================================================ STEER: headlight
def headlight_parts():
    """Returns (list of parts in headlight-local coords, emitter object). Local Z = beam axis."""
    parts = []
    # shell: thin-walled bucket
    N = 18
    outer = [(0.0905, 0.0020), (0.0915, -0.0040)]
    for k in range(N + 1):
        ph = (PI / 2) * k / N
        outer.append((0.0915 * math.cos(ph), -0.012 - 0.128 * math.sin(ph)))
    inner = []
    for k in range(N, -1, -1):
        ph = (PI / 2) * k / N
        inner.append((0.0900 * math.cos(ph), -0.012 - 0.1265 * math.sin(ph)))
    inner.append((0.0900, 0.0020))
    shell = lathe("hshell", outer + inner, segs=96, closed=True, mat="alu_turned")
    for s in (1, -1):
        parts.append(cylz("hboss", 0.0075, 0.0835, 0.0916, M=frame((0, 0, -0.030), (0, s, 0)), segs=32,
                          bevel=0.0005, mat="alu_turned"))
    parts.append(shell)
    # chrome bezel
    bez = [(0.0895, -0.0100), (0.0935, -0.0060), (0.0955, 0.0000), (0.0961, 0.0100), (0.0952, 0.0200),
           (0.0924, 0.0272), (0.0884, 0.0300), (0.0848, 0.0291), (0.0832, 0.0266), (0.0834, 0.0232),
           (0.0858, 0.0185), (0.0878, 0.0050), (0.0878, -0.0060)]
    parts.append(lathe("hbezel", bez, segs=96, closed=True, mat="chrome", smooth=50.0))
    for th in (0.0, 2.0944, -2.0944):
        rd = Vector((math.cos(th), math.sin(th), 0.0))
        parts.append(bolt("hscr", 0.0030, rd * 0.0958 + Vector((0, 0, 0.0105)), rd, kind="button",
                          mat="steel_polished", x_hint=(0, 0, 1)))
    vis = annular_sector(0.0941, 0.0960, PI - rad(78), PI + rad(78), 48)
    parts.append(prism("hvisor", vis, 0.0255, 0.0415, mat="chrome"))
    # lens: domed outside, fresnel steps inside
    def zo(r):
        return 0.0300 - 0.0068 * (r / 0.0855) ** 2
    lens = [(0.0, zo(0.0))]
    for k in range(1, 13):
        r = 0.0855 * k / 12
        lens.append((r, zo(r)))
    lens.append((0.0855, 0.0200))
    rings = 11
    for k in range(rings, 0, -1):
        r_hi = 0.0840 * k / rings
        r_lo = 0.0840 * (k - 1) / rings + 0.0002
        lens.append((r_hi, zo(r_hi) - 0.0034))
        lens.append((r_lo + 0.0001, zo(r_lo) - 0.0026))
    lens.append((0.0, zo(0.0) - 0.0028))
    parts.append(lathe("hlens", lens, segs=96, mat="glass_lens", smooth=60.0))
    # parabolic reflector
    z0, fl = -0.055, 0.026
    refl = []
    for k in range(25):
        r = 0.0105 + (0.0858 - 0.0105) * k / 24
        refl.append((r, z0 + r * r / (4 * fl)))
    for k in range(24, -1, -1):
        r = 0.0105 + (0.0858 - 0.0105) * k / 24
        refl.append((r, z0 + r * r / (4 * fl) - 0.0012))
    parts.append(lathe("hrefl", refl, segs=96, closed=True, mat="mirror", smooth=60.0))
    # bulb socket behind reflector + chrome bulb cap
    parts.append(lathe("hsock", [(0.0, -0.078), (0.0125, -0.078), (0.0135, -0.076), (0.0135, -0.0545),
                                 (0.0105, -0.0535), (0.0, -0.0535)], segs=40, mat="ano_black"))
    parts.append(lathe("hcap", [(0.0, -0.0060), (0.0050, -0.0066), (0.0074, -0.0090), (0.0080, -0.0130),
                                (0.0076, -0.0156), (0.0066, -0.0156), (0.0069, -0.0128), (0.0063, -0.0095),
                                (0.0045, -0.0082), (0.0, -0.0078)], segs=40, closed=False, mat="chrome", smooth=60.0))
    # emitter (bulb)
    bp = [(0.0, -0.0535), (0.0045, -0.0535), (0.0062, -0.0450), (0.0066, -0.0300), (0.0062, -0.0215),
          (0.0050, -0.0175), (0.0030, -0.0160), (0.0, -0.0156)]
    emit = lathe("headlight_emit", bp, segs=32, mat="emit_head", smooth=70.0)
    return parts, emit


# ============================================================================ STEER: gauges
def gauge_parts(s, G):
    """Gauge parts transformed into F; returns (static parts, needle object, needle frame)."""
    parts = []
    cup = [(0.0, -0.034), (0.020, -0.034), (0.024, -0.033), (0.027, -0.030), (0.0295, -0.022), (0.031, -0.012),
           (0.0318, -0.003), (0.0318, 0.0005), (0.0, 0.0005)]
    parts.append(lathe("gcup", cup, M=G, segs=64, mat="ano_black"))
    bez = [(0.0292, -0.0030), (0.0335, -0.0030), (0.0350, -0.0010), (0.0353, 0.0030), (0.0346, 0.0065),
           (0.0331, 0.0085), (0.0311, 0.0091), (0.0298, 0.0083), (0.0292, 0.0070)]
    parts.append(lathe("gbez", bez, M=G, segs=64, closed=True, mat="chrome", smooth=50.0))
    parts.append(lathe("gface", [(0.0, 0.0), (0.0294, 0.0), (0.0294, 0.0008), (0.0, 0.0008)], M=G, segs=64,
                       mat="gauge_face"))
    parts.append(lathe("gtun", [(0.0279, 0.0008), (0.0294, 0.0008), (0.0294, 0.0049), (0.0279, 0.0049)], M=G, segs=64,
                       closed=True, mat="chrome"))
    parts.append(lathe("ghub", [(0.0, 0.0008), (0.0042, 0.0008), (0.0042, 0.0011), (0.0, 0.0011)], M=G, segs=24,
                       mat="ano_black"))
    for th in (60.0, 180.0, 300.0):
        tr = rad(th)
        parts.append(bolt("gscr", 0.0022, G @ Vector((0.0323 * math.sin(tr), 0.0323 * math.cos(tr), 0.0086)),
                          G.to_3x3() @ Vector((0, 0, 1)), kind="button", mat="steel_polished"))
    parts.append(lathe("gglass", [(0.0, 0.0062), (0.015, 0.0060), (0.0298, 0.0052), (0.0298, 0.0040),
                                  (0.015, 0.0048), (0.0, 0.0050)], M=G, segs=64, mat="glass", smooth=60.0))
    # tick marks (white), arc ring, redline band (orange)
    ticks = []
    nt = 28
    for i in range(nt):
        th = rad(-135.0 + 270.0 * i / (nt - 1))
        major = (i % 3 == 0)
        L = 0.0046 if major else 0.0026
        w = 0.0009 if major else 0.00048
        rc = 0.0258 - L / 2
        R = Matrix.Rotation(-th, 4, "Z")
        tk = box("tick", (-w / 2, rc - L / 2, 0.0008), (w / 2, rc + L / 2, 0.00115), M=G @ R)
        ticks.append(tk)
    arc = []
    for k in range(41):
        th = rad(-135.0 + 270.0 * k / 40)
        arc.append((0.0262 * math.sin(th), 0.0262 * math.cos(th)))
    for k in range(40, -1, -1):
        th = rad(-135.0 + 270.0 * k / 40)
        arc.append((0.0267 * math.sin(th), 0.0267 * math.cos(th)))
    ticks.append(prism("garc", arc, 0.0008, 0.00112, M=G))
    for tk in ticks:
        setmat(tk, "white_paint")
    parts += ticks
    if s > 0:
        red = []
        for k in range(9):
            th = rad(95.0 + 40.0 * k / 8)
            red.append((0.0238 * math.sin(th), 0.0238 * math.cos(th)))
        for k in range(8, -1, -1):
            th = rad(95.0 + 40.0 * k / 8)
            red.append((0.0256 * math.sin(th), 0.0256 * math.cos(th)))
        parts.append(prism("gred", red, 0.0008, 0.0011, M=G, mat="ano_orange"))
    # needle (separate, rest pose pointing at the zero mark, -135 deg)
    NM = G @ T(0, 0, 0.0014)
    rz = Matrix.Rotation(rad(135.0), 4, "Z")
    nd = prism("needle", [(-0.0003, 0.0225), (-0.0009, 0.0), (-0.0012, -0.0055), (0.0012, -0.0055),
                          (0.0009, 0.0), (0.0003, 0.0225)], 0.0, 0.0005, M=rz, mat="ano_orange")
    hub = lathe("nhub", [(0.0, -0.0004), (0.0023, -0.0004), (0.0023, 0.0009), (0.0018, 0.0013), (0.0, 0.0013)],
                segs=24, mat="ano_black")
    needle = join([nd, hub], "gauge_needle_" + ("L" if s > 0 else "R"))
    needle.matrix_basis = NM
    return parts, needle


# ============================================================================ LOWER: caliper
def caliper_matrix(s):
    eu = Vector((math.cos(CAL_PHI), 0, math.sin(CAL_PHI)))
    ev = -s * Vector((-math.sin(CAL_PHI), 0, math.cos(CAL_PHI)))
    ew = Vector((0, s, 0))
    return basis(Vector((TX, s * C.DISC_F_Y, WC.z)), eu, ev, ew)


def caliper_outline():
    pts = []
    for k in range(25):
        psi = -0.30 + 0.60 * k / 24
        pts.append((0.178 * math.cos(psi), 0.178 * math.sin(psi)))
    for sv in (1, -1):
        pts += circ(0.160 * math.cos(0.33), sv * 0.160 * math.sin(0.33), 0.018, 40)
        pts += circ(0.1035, sv * 0.058, 0.0095, 32)
    return hull2d(pts)


def weight_bevel(ob, mat_name, width, segs=4, angle=30.0):
    """Big rounded edges only on faces of one material (via edge bevel weights) - placed before the
    regular angle bevel added by finish()."""
    me = ob.data
    idx = [i for i, m in enumerate(me.materials) if m is not None and m.name == mats.get(mat_name).name]
    if not idx:
        return None
    idx = idx[0]
    bm = bmesh.new()
    bm.from_mesh(me)
    lay = bm.edges.layers.float.get("bevel_weight_edge") or bm.edges.layers.float.new("bevel_weight_edge")
    lim = rad(angle)
    for e in bm.edges:
        w = 0.0
        if len(e.link_faces) == 2 and all(f.material_index == idx for f in e.link_faces):
            if e.calc_face_angle(0.0) > lim:
                w = 1.0
        e[lay] = w
    bm.to_mesh(me)
    bm.free()
    m = ob.modifiers.new("BevelBig", "BEVEL")
    m.limit_method = "WEIGHT"
    m.width = width
    m.segments = segs
    try:
        m.use_clamp_overlap = True
        m.harden_normals = False
    except Exception:
        pass
    return m


def caliper(s, K, parts):
    """Radial four-piston monobloc caliper in caliper-local coords (u radial, v tangential, w axial),
    transformed by K into F.  Appends finished pieces to parts."""
    vt = 1.0 if s > 0 else -1.0      # sign of v towards the top end (see caliper_matrix)
    body = prism("cal", caliper_outline(), -0.024, 0.024)
    # piston bulges on both faces (outer ones get machined caps below)
    bul = []
    for sv in (1, -1):
        for sw in (1, -1):
            w0, w1 = sorted((sw * 0.019, sw * 0.0268))
            bul.append(cylz("bul", 0.0168, w0, w1, M=T(0.1355, sv * 0.0215, 0), segs=48))
    union(body, bul)
    VW = basis((0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0))     # local (v, w, u) -> (u, v, w)
    cs = [cylz("c", 0.1615, -0.0035, 0.0035, segs=160),
          prism("c", rrect(0.0, 0.0, 0.032, 0.0105, 0.0105), 0.1485, 0.21, M=VW)]
    for sv in (1, -1):
        cs.append(cylz("c", 0.0088, 0.1085, 0.131, M=frame((0, sv * 0.058, 0.016), (1, 0, 0), (0, 0, 1)), segs=32))
        cs.append(cylz("c", 0.0045, 0.07, 0.12, M=frame((0, sv * 0.058, 0.016), (1, 0, 0), (0, 0, 1)), segs=20))
    cut(body, cs)
    setmat(body, "ano_black")
    pieces = [body]
    # machined faces on the outer piston bulges
    for sv in (1, -1):
        pieces.append(lathe("pcap", [(0.0, 0.0263), (0.0128, 0.0263), (0.0132, 0.0267), (0.0132, 0.0271),
                                     (0.0, 0.0271)], M=T(0.1355, sv * 0.0215, 0), segs=48, mat="alu_machined"))
    # pads (friction + backing), visible through the window
    pad = []
    for k in range(13):
        psi = -0.27 + 0.54 * k / 12
        pad.append((0.1535 * math.cos(psi), 0.1535 * math.sin(psi)))
    for k in range(12, -1, -1):
        psi = -0.27 + 0.54 * k / 12
        pad.append((0.117 * math.cos(psi), 0.117 * math.sin(psi)))
    pad = fillet_poly(pad, 0.004, 4)
    for sw in (1, -1):
        pieces.append(prism("padf", pad, sw * 0.0036, sw * 0.0076, mat="engine_fin"))
        pieces.append(prism("padb", pad, sw * 0.0076, sw * 0.0106, mat="steel_dark"))
    # pad pin across the window
    pieces.append(cylz("pin", 0.0021, -0.047, 0.047, M=frame((0.1625, 0, 0.0), (0, 1, 0), (1, 0, 0)), segs=20,
                       bevel=0.0004, mat="steel_polished"))
    # radial mount bolts (bright)
    for sv in (1, -1):
        pieces.append(bolt("calb", 0.010, (0.1085, sv * 0.058, 0.016), (1, 0, 0), head_h=0.0085,
                           mat="steel_polished", x_hint=(0, 0, 1)))
    # bleed nipple at the top end
    bz = 0.0145
    Fb = frame((0.160, 0, bz), (0, vt, 0), (1, 0, 0))
    pieces.append(cylz("bleedhex", 0.0042, 0.063, 0.0715, M=Fb, segs=6, bevel=0.0004, mat="steel_polished"))
    pieces.append(cylz("bleed", 0.0026, 0.0715, 0.0790, r2=0.0017, M=Fb, segs=20, mat="steel_polished"))
    pieces.append(lathe("bleedcap", [(0.0, 0.0812), (0.0030, 0.0812), (0.0036, 0.0800), (0.0036, 0.0752),
                                     (0.0, 0.0752)], M=Fb, segs=24, mat="rubber"))
    # banjo fitting + short hose stub at the top end
    ub = 0.142
    Fj = frame((ub, 0, bz), (0, vt, 0), (1, 0, 0))
    # double banjo (feed line + crossover) with copper crush washers
    for (a0, a1, mt, sg, r) in ((0.0662, 0.0678, "copper", 32, 0.0080), (0.0678, 0.0778, "steel_polished", 32, 0.0075),
                                (0.0778, 0.0790, "copper", 32, 0.0080), (0.0790, 0.0890, "steel_polished", 32, 0.0075),
                                (0.0890, 0.0902, "copper", 32, 0.0080), (0.0902, 0.0962, "steel_polished", 6, 0.0068)):
        pieces.append(cylz("banjo", r, a0, a1, M=Fj, segs=sg, bevel=0.0005 if a1 - a0 > 0.003 else 0.0, mat=mt))
    ends = []
    for vc in (0.0728, 0.0840):
        Fn = frame((0.0, vt * vc, bz), (-1, 0, 0), (0, 1, 0))
        pieces.append(cylz("neck", 0.0042, -ub, -0.1285, M=Fn, segs=24, mat="steel_polished"))
        pieces.append(cylz("ferr", 0.0050, -0.1285, -0.1155, M=Fn, segs=24, bevel=0.0005, mat="steel_polished"))
        ends.append((K @ Vector((0.1155, vt * vc, bz)), (K.to_3x3() @ Vector((-1, 0, 0))).normalized()))
    for p in pieces:
        xform(p, K)
    parts += pieces
    return ends


def fork_foot(s, K, studs):
    fv = lathe("foot", [(0.0, -0.7395), (0.0265, -0.7395), (0.0285, -0.7375), (0.0285, -0.6440),
                        (0.0272, -0.6405), (0.0226, -0.6400), (0.0, -0.6400)], M=T(TX, s * TY, 0), segs=72)
    y0, y1 = sorted((s * 0.0835, s * 0.1345))
    boss = cylz("boss", 0.0265, y0, y1, M=T(TX, 0, WC.z) @ XZ, segs=64)
    pinch = box("pinch", (0.040, s * 0.0845, -0.737), (0.0655, s * 0.1335, -0.683))
    tab = box("tab", (0.050, s * 0.0865, -0.677), (0.069, s * 0.1005, -0.659))
    # caliper lug bosses (radial), and the web connecting them to the foot
    A = K @ Vector((0.0825, 0.058, 0.016))
    B = K @ Vector((0.0825, -0.058, 0.016))
    lugs = [cylz("lug", 0.0085, 0.0735, 0.0943, M=K @ frame((0, sv * 0.058, 0.016), (1, 0, 0), (0, 0, 1)), segs=32)
            for sv in (1, -1)]
    web_pts = hull2d(circ(TX - 0.006, WC.z + 0.004, 0.022, 48) + circ(A.x, A.z, 0.0088, 32) +
                     circ(B.x, B.z, 0.0088, 32))
    wy0, wy1 = sorted((s * 0.0868, s * 0.1032))
    web = prism("web", web_pts, wy0, wy1, M=XZ)
    union(fv, [boss, pinch, tab, web] + lugs)
    cs = []
    iy0, iy1 = sorted((s * 0.0835, -s * 0.2))
    cs.append(box("c", (-0.2, iy0, -0.9), (0.2, iy1, -0.5)))
    cs.append(cylz("c", 0.0112, -0.2, 0.2, M=T(TX, 0, WC.z) @ XZ, segs=48))
    cs.append(box("c", (TX, -0.2, WC.z - 0.0006), (0.09, 0.2, WC.z + 0.0006)))
    for yy in (0.096, 0.122):
        cs.append(cylz("c", 0.0026, -0.76, -0.69, M=T(0.0545, s * yy, 0), segs=20))
        cs.append(cylz("c", 0.0050, -0.76, -0.7315, M=T(0.0545, s * yy, 0), segs=28))
        studs.append(bolt("pb", 0.006, (0.0545, s * yy, -0.7315), (0, 0, -1), mat="steel_polished"))
    cs.append(cylz("c", 0.0025, 0.04, 0.08, M=frame((0, s * 0.0935, -0.668), (1, 0, 0)), segs=20))
    for sv in (1, -1):
        cs.append(cylz("c", 0.0045, 0.06, 0.095, M=K @ frame((0, sv * 0.058, 0.016), (1, 0, 0), (0, 0, 1)), segs=20))
    cut(fv, cs)
    return fv


def stanchion(s):
    return lathe("stanch", [(0.0, -0.220), (0.0205, -0.220), (0.0215, -0.2212), (0.0215, -0.6450), (0.0, -0.6450)],
                 M=T(TX, s * TY, 0), segs=64, mat="chrome", smooth=50.0)


def axle_parts():
    P = []
    M = T(TX, 0, WC.z) @ XZ          # local z -> F y
    # XZ is a reflection; lathe along local z then mapped onto F.y
    P.append(cylz("axle", 0.011, -0.1395, 0.1360, M=M, segs=40, mat="steel_polished"))
    P.append(lathe("axhead", [(0.0, 0.1345), (0.0170, 0.1345), (0.0176, 0.1352), (0.0176, 0.1392),
                              (0.0165, 0.1408), (0.0082, 0.1408), (0.0082, 0.1372), (0.0, 0.1372)], M=M, segs=48,
                   mat="steel_polished"))
    P.append(cylz("axnut", 0.0158, -0.1455, -0.1345, M=M, segs=6, bevel=0.0010, mat="steel_polished"))
    P.append(lathe("axend", [(0.0, -0.1478), (0.0098, -0.1478), (0.0110, -0.1466), (0.0110, -0.1450),
                             (0.0, -0.1450)], M=M, segs=40, mat="steel_polished"))
    return P


def fender_parts():
    """Rolled-edge fender in the fender frame (origin wheel centre, world-aligned axes)."""
    th0, th1 = rad(46.5), rad(160.0)
    t = 0.0015
    Y = 0.0695
    rb = 0.0028

    def rin(y):
        return 0.336 - 0.022 * (y / 0.072) ** 2

    sec = []
    ny = 46
    # outer surface (left -> right) with a small raised centre rib
    for k in range(ny + 1):
        y = -Y + 2 * Y * k / ny
        sec.append((y, rin(y) + t + 0.0011 * math.exp(-(y / 0.0042) ** 2)))
    # right bead
    cy, cr = Y + 0.0008, rin(Y) + t * 0.5
    for k in range(1, 12):
        a = PI / 2 - PI * k / 12
        sec.append((cy + rb * math.cos(a) * 0.95, cr + rb * math.sin(a)))
    # inner surface (right -> left)
    for k in range(ny, -1, -1):
        y = -Y + 2 * Y * k / ny
        sec.append((y, rin(y)))
    cy = -Y - 0.0008
    for k in range(1, 12):
        a = -PI / 2 - PI * k / 12
        sec.append((cy + rb * math.cos(a) * 0.95, cr + rb * math.sin(a)))
    bm = bmesh.new()
    nth = 72
    rings = []
    for j in range(nth + 1):
        th = th0 + (th1 - th0) * j / nth
        lip = max(0.0, (rad(50.0) - th) / rad(3.5))          # thin flared front lip
        sc = 1.0 + 0.016 * lip ** 2
        ring = [bm.verts.new((rho * sc * math.cos(th), y, rho * sc * math.sin(th))) for (y, rho) in sec]
        rings.append(ring)
    m = len(sec)
    for j in range(nth):
        for i in range(m):
            i2 = (i + 1) % m
            bm.faces.new((rings[j][i], rings[j][i2], rings[j + 1][i2], rings[j + 1][i]))
    bm.faces.new(rings[0])
    bm.faces.new(rings[-1][::-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    fender = mk("fender", bm, "alu_plate", smooth=50.0)
    return fender


def fender_stays(s, Finv):
    """Stay rods (F coords) with eye at the foot and a flange + button head at the fender,
    returned transformed into the fender frame."""
    P = []
    x = 0.0705

    def zr(rho):
        return WC.z + math.sqrt(rho ** 2 - (x - TX) ** 2)

    rhat = Vector((x - TX, 0.0, zr(0.31) - WC.z)).normalized()
    ya = 0.0625
    rin = 0.336 - 0.022 * (ya / 0.072) ** 2
    slope = -0.044 * ya / 0.072 ** 2
    nloc = Vector((-slope, 1.0)).normalized()                  # (y, rho) normal of the inner surface
    n_att = rhat * nloc.y + Vector((0, s * nloc.x, 0))
    Q = WC + rhat * rin + Vector((0, s * ya, 0))               # attach point on the inner surface
    P4 = Q - n_att * 0.0032
    pts = [Vector((x, s * 0.0935, -0.6680)), Vector((x, s * 0.0935, -0.5450)), Vector((x, s * 0.0715, -0.5150)),
           Vector((x, s * 0.0715, zr(0.292))), P4]
    rod = geo.tube("stay", pts, 0.0034, segs=20, fillet=0.016, col=_col())
    setmat(rod, "steel_polished")
    P.append(rod)
    # flattened eye at the foot tab + button bolt
    P.append(cylz("eye", 0.0062, 0.0690, 0.0730, M=frame((0, s * 0.0935, -0.668), (1, 0, 0)), segs=28,
                  bevel=0.0006, mat="steel_polished"))
    P.append(bolt("eyeb", 0.005, (0.0730, s * 0.0935, -0.668), (1, 0, 0), kind="button", mat="steel_polished"))
    # welded flange under the fender, button head on top
    P.append(lathe("flange", [(0.0, -0.0042), (0.0058, -0.0042), (0.0066, -0.0034), (0.0066, 0.0002),
                              (0.0, 0.0002)], M=frame(Q, n_att), segs=28, mat="steel_polished"))
    P.append(lathe("rivet", [(0.0, 0.0034), (0.0016, 0.0031), (0.0030, 0.0022), (0.0038, 0.0010),
                             (0.0040, 0.0010), (0.0040, 0.0008), (0.0, 0.0008)], M=frame(Q, n_att), segs=24,
                   mat="steel_polished", smooth=70.0))
    for p in P:
        xform(p, Finv)
    return P


# ============================================================================ CLIP-ON
def clipon_frame(s):
    d = Vector((-math.sin(rad(8)) * math.cos(rad(6)), s * math.cos(rad(8)) * math.cos(rad(6)), -math.sin(rad(6))))
    g = Vector((math.sin(RAKE), 0, math.cos(RAKE)))      # world up in F
    n = (g - d * g.dot(d)).normalized()
    f = d.cross(n)
    if f.x < 0:
        f = -f
    f.normalize()
    return d, f, n


def grip_obj(name, L, r0, M):
    nseg, nlen = 72, 84
    bm = bmesh.new()
    rings = []
    for j in range(nlen + 1):
        t = L * j / nlen
        fade = min(1.0, t / 0.004, (L - t) / 0.004)
        ring = []
        for i in range(nseg):
            a = 2 * PI * i / nseg
            u = 12 * a / (2 * PI) + t / 0.0085
            v = 12 * a / (2 * PI) - t / 0.0085
            tu = 1 - 2 * abs((u % 1.0) - 0.5)
            tv = 1 - 2 * abs((v % 1.0) - 0.5)
            h = min(1.0, 2.2 * min(tu, tv))
            r = r0 + 0.0008 * h * max(0.0, fade) - (0.0006 * (1 - max(0.0, fade)))
            ring.append(bm.verts.new((r * math.cos(a), r * math.sin(a), t)))
        rings.append(ring)
    for j in range(nlen):
        for i in range(nseg):
            i2 = (i + 1) % nseg
            bm.faces.new((rings[j][i], rings[j][i2], rings[j + 1][i2], rings[j + 1][i]))
    bm.faces.new(rings[0][::-1])
    bm.faces.new(rings[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    ob = mk(name, bm, "rubber", smooth=60.0)
    return xform(ob, M)


def sweep_xy(name, path, widths, thicks, nsec=16, exp=3.2, mat=None):
    P = [Vector((p[0], p[1])) for p in path]
    N = len(P)
    bm = bmesh.new()
    rings = []
    for i, p in enumerate(P):
        if i == 0:
            tg = P[1] - P[0]
        elif i == N - 1:
            tg = P[-1] - P[-2]
        else:
            tg = P[i + 1] - P[i - 1]
        tg.normalize()
        nr = Vector((-tg.y, tg.x))
        f = i / (N - 1)
        w = (widths[0] + (widths[1] - widths[0]) * f) / 2
        h = (thicks[0] + (thicks[1] - thicks[0]) * f) / 2
        ring = []
        for k in range(nsec):
            a = 2 * PI * k / nsec
            ca, sa = math.cos(a), math.sin(a)
            cx = math.copysign(abs(ca) ** (2 / exp), ca) * w
            cz = math.copysign(abs(sa) ** (2 / exp), sa) * h
            ring.append(bm.verts.new((p.x + nr.x * cx, p.y + nr.y * cx, cz)))
        rings.append(ring)
    for i in range(N - 1):
        for k in range(nsec):
            k2 = (k + 1) % nsec
            bm.faces.new((rings[i][k], rings[i][k2], rings[i + 1][k2], rings[i + 1][k]))
    bm.faces.new(rings[0][::-1])
    bm.faces.new(rings[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return mk(name, bm, mat, smooth=60.0)


def clipon(s, root, tag):
    d, f, n = clipon_frame(s)
    Mb = basis((0, 0, 0), d, f, n)              # (t along bar, forward, up) -> clip-on local

    def B(t):
        return d * t

    body = []
    # ---- clamp ring with rear pinch ear and bar boss (machined)
    ring = cylz("cl", 0.0365, -0.020, 0.020, segs=72)
    ear = box("ear", (-0.053, -0.012, -0.020), (-0.030, 0.012, 0.020))
    boss = lathe("boss", [(0.0, 0.012), (0.0155, 0.012), (0.0155, 0.0585), (0.0142, 0.0615), (0.0, 0.0615)],
                 M=frame((0, 0, 0), d, f), segs=48)
    union(ring, [ear, boss])
    cs = [cylz("c", 0.0272, -0.03, 0.03, segs=72),
          box("c", (-0.060, -0.0006, -0.03), (-0.026, 0.0006, 0.03))]
    for zz in (-0.0095, 0.0095):
        cs.append(cylz("c", 0.0024, -0.02, 0.02, M=frame((-0.042, 0, zz), (0, 1, 0)), segs=20))
        body.append(bolt("cb", 0.005, (-0.042, 0.012, zz), (0, 1, 0), washer=True))
    cut(ring, cs)
    setmat(ring, "alu_machined")
    body.append(ring)
    # ---- bar tube
    body.append(lathe("bar", [(0.0, 0.040), (0.011, 0.040), (0.011, 0.2890), (0.0, 0.2890)],
                      M=frame((0, 0, 0), d, f), segs=40, mat="ano_black"))
    # ---- master cylinder (radial) with perch, reservoir base
    Fd = frame((0, 0, 0), d, f)
    body.append(lathe("mcr", [(0.0108, 0.092), (0.0166, 0.092), (0.0175, 0.0930), (0.0175, 0.1110),
                              (0.0166, 0.1120), (0.0108, 0.1120)], M=Fd, segs=40, closed=True, mat="ano_black"))
    body.append(box("mcb", (0.090, 0.004, -0.0105), (0.120, 0.046, 0.0105), M=Mb, bevel=0.0035, segs=3,
                    mat="ano_black"))
    body.append(cylz("mcbar", 0.0105, 0.0, 0.0475, M=frame(B(0.105) + n * (-0.001), f, d), segs=32, bevel=0.0015,
                     mat="ano_black"))
    body.append(box("mccap", (0.092, -0.0235, -0.0125), (0.112, -0.0085, 0.0125), M=Mb, bevel=0.003, segs=3,
                    mat="ano_black"))
    for nn in (-0.0072, 0.0072):
        body.append(bolt("mcbolt", 0.005, B(0.102) + f * (-0.0235) + n * nn, -f, washer=True))
    for sn in (1, -1):
        lo, hi = sorted((sn * 0.0042, sn * 0.0098))
        body.append(box("lug", (0.116, 0.026, lo), (0.140, 0.053, hi), M=Mb, bevel=0.0018, segs=3,
                        mat="alu_machined"))
    P = B(0.130) + f * 0.0435
    body.append(cylz("pivot", 0.0030, -0.0112, 0.0112, M=frame(P, n), segs=20, mat="steel_dark"))
    for sn in (1, -1):
        body.append(cylz("pivh", 0.0046, sn * 0.0098, sn * 0.0114, M=frame(P, n), segs=6, bevel=0.0003,
                         mat="steel_dark"))
    Rc = B(0.100) + f * 0.012
    FR = frame(Rc, n, d)
    body.append(cylz("stalk", 0.0058, 0.006, 0.0215, M=FR, segs=24, mat="ano_black"))
    body.append(lathe("rbase", [(0.0, 0.0200), (0.0176, 0.0200), (0.0192, 0.0214), (0.0192, 0.0245),
                                (0.0, 0.0245)], M=FR, segs=48, mat="ano_black"))
    body.append(lathe("rcap", [(0.0, 0.0592), (0.0172, 0.0592), (0.0193, 0.0582), (0.0197, 0.0565),
                               (0.0197, 0.0538), (0.0188, 0.0538), (0.0188, 0.0552), (0.0, 0.0552)], M=FR, segs=48,
                      mat="plastic_black"))
    body.append(lathe("rring", [(0.0186, 0.0516), (0.0196, 0.0516), (0.0198, 0.0526), (0.0198, 0.0536),
                                (0.0186, 0.0536)], M=FR, segs=48, closed=True, mat="ano_orange"))
    # ---- switch pod (left) / throttle housing (right): two half shells with a parting line
    hs = 0.0003
    body.append(box("swu", (0.136, -0.021, hs), (0.168, 0.022, 0.0205), M=Mb, bevel=0.0055, segs=3,
                    mat="plastic_black"))
    body.append(box("swl", (0.136, -0.021, -0.019), (0.168, 0.022, -hs), M=Mb, bevel=0.0055, segs=3,
                    mat="plastic_black"))
    if s > 0:
        btns = [((0.141, -0.0245, 0.004), (0.152, -0.0195, 0.012)),
                ((0.153, -0.0245, -0.012), (0.163, -0.0195, -0.004)),
                ((0.146, -0.012, 0.0190), (0.158, -0.004, 0.0235))]
    else:
        btns = [((0.143, -0.0245, 0.003), (0.161, -0.0195, 0.013)),
                ((0.146, -0.0245, -0.012), (0.158, -0.0195, -0.004))]
        cd = (-d * 0.62 + f * 0.62 - n * 0.22).normalized()
        cp = B(0.146) + f * 0.012 - n * 0.010
        body.append(cylz("tout", 0.0050, 0.0, 0.016, M=frame(cp, cd), segs=24, bevel=0.0008, mat="plastic_black"))
        body.append(cylz("tadj", 0.0036, 0.016, 0.0235, M=frame(cp, cd), segs=6, bevel=0.0004, mat="alu_machined"))
        cp2 = B(0.1585) + f * 0.012 - n * 0.010
        body.append(cylz("tout2", 0.0050, 0.0, 0.016, M=frame(cp2, cd), segs=24, bevel=0.0008, mat="plastic_black"))
        body.append(cylz("tadj2", 0.0036, 0.016, 0.0235, M=frame(cp2, cd), segs=6, bevel=0.0004, mat="alu_machined"))
        throttle_starts = [cp + cd * 0.0235, cp2 + cd * 0.0235]
    for a, b in btns:
        body.append(box("btn", a, b, M=Mb, bevel=0.0016, segs=2, mat="rubber"))
    # ---- grip + flange
    body.append(lathe("gfl", [(0.0110, 0.1705), (0.0200, 0.1705), (0.0207, 0.1715), (0.0207, 0.1745),
                              (0.0195, 0.1757), (0.0163, 0.1768), (0.0110, 0.1768)], M=Fd, segs=48, closed=True,
                      mat="rubber"))
    grip = grip_obj("clipon_%s_grip" % tag, 0.287 - 0.1765, 0.0160, None)
    grip.matrix_basis = frame(B(0.1765), d, f)
    Mbe = frame(B(0.287), d, f)
    lines = []
    # ---- hydraulic line: banjo on the MC top -> gentle loop -> end fitting above the upper clamp
    Pb = B(0.114) + f * 0.040 + n * 0.0105
    bj, cen = banjo_stack(Pb, n, d, 0.0, 1)
    body += bj
    eye = Pb + n * cen[0]
    body.append(cylz("bjn", 0.0042, 0.0, 0.013, M=frame(eye, -d, f), segs=20, mat="steel_polished"))
    body.append(cylz("bjf", 0.0049, 0.013, 0.025, M=frame(eye, -d, f), segs=24, bevel=0.0005, mat="steel_polished"))
    h0 = eye - d * 0.025
    E = Vector((0.045, -0.012 * s, 0.074))
    body += hose_end(E, (0, 0, -1), "ano_blue" if s > 0 else "ano_red")
    hp = [h0, h0 - d * 0.010 + n * 0.002, B(0.058) + f * 0.052 + n * 0.042, Vector((0.074, 0.040 * s, 0.068)),
          Vector((0.062, 0.012 * s, 0.097)), Vector((0.049, -0.006 * s, 0.104)), E + Vector((0, 0, 0.026)),
          E + Vector((0, 0, 0.012))]
    lines.append(line("hyd_" + tag, hp, 0.0035, sub=10))
    # ---- booted junction near the clamp for switch wiring (+ throttle cables on the right)
    J = Vector((0.044, 0.078 * s, -0.014))
    jd = Vector((0.15, 0.30 * s, -1.0)).normalized()
    FJ = frame(J, jd, f)
    body.append(lathe("boot", [(0.0, -0.0010), (0.0088, -0.0010), (0.0092, 0.0030), (0.0084, 0.0060),
                               (0.0090, 0.0090), (0.0080, 0.0125), (0.0084, 0.0155), (0.0070, 0.0200),
                               (0.0062, 0.0230), (0.0, 0.0230)], M=FJ, segs=32, mat="rubber"))
    body.append(cylz("bootl", 0.0056, 0.020, 0.033, M=FJ, segs=24, mat="rubber"))
    body.append(cylz("bootc", 0.0060, 0.033, 0.0352, M=FJ, segs=24, bevel=0.0004, mat="plastic_black"))
    wp = [B(0.150) - f * 0.006 - n * 0.016, B(0.140) - f * 0.004 - n * 0.0235, B(0.100) - f * 0.002 - n * 0.0255,
          B(0.066) + f * 0.010 - n * 0.0245, B(0.070) + f * 0.032 - n * 0.016, J + Vector((0.0, 0.0, 0.018)) + jd * -0.004,
          J + jd * 0.004]
    lines.append(line("sw_" + tag, wp, 0.0032, mat="rubber", segs=16))
    for tt in (0.120, 0.080):
        c = B(tt) - f * 0.002 - n * 0.0255
        body += tie(c, d, 0.0034)
    if s < 0:
        for k, st in enumerate(throttle_starts):
            off = n * (0.004 * (k - 0.5))
            cpts = [st, st + cd * 0.010, B(0.102) + f * 0.040 - n * 0.020 + off, B(0.066) + f * 0.030 - n * 0.016 + off,
                    J + Vector((0.004 * (k - 0.5), 0.0, 0.020)), J + Vector((0.003 * (k - 0.5), 0, 0.006))]
            lines.append(line("thr%d_%s" % (k, tag), cpts, 0.0027, mat="plastic_black", segs=12))
    # ---- grip lock ring + safety wire
    body.append(lathe("lockr", [(0.0158, 0.0), (0.0182, 0.0), (0.0187, 0.0005), (0.0187, 0.0068), (0.0182, 0.0073),
                                (0.0158, 0.0073)], M=frame(B(0.2795), d, f), segs=48, closed=True, mat="alu_machined"))
    body.append(bolt("lrs", 0.0028, B(0.2832) + n * 0.0187, n, washer=False))
    for tt in (0.188, 0.271):
        body += twisted_loop(B(tt), d, f, n, 0.0170)
    # ---- bar-end mirror (machined stem, knuckle, small round head folded flat at the bar end)
    body.append(lathe("mstem", [(0.0, 0.0), (0.0056, 0.0), (0.0056, 0.0125), (0.0050, 0.0140), (0.0, 0.0140)],
                      M=frame(B(0.3075), d, f), segs=32, mat="alu_machined"))
    kn = B(0.3225)
    body.append(sphere("mknuck", 0.0061, M=T(kn), mat="alu_machined"))
    Hm = B(0.3305) + f * 0.008 + n * 0.010
    av = (Hm - d * 0.0035) - kn
    body.append(cylz("marm", 0.0034, 0.0, av.length, M=frame(kn, av), segs=20, mat="alu_machined"))
    FH = frame(Hm, d, f)
    body.append(lathe("mhead", [(0.0, -0.0045), (0.0140, -0.0042), (0.0200, -0.0031), (0.0232, -0.0010),
                                (0.0236, 0.0018), (0.0230, 0.0032), (0.0216, 0.0032), (0.0216, 0.0018), (0.0, 0.0018)],
                      M=FH, segs=64, mat="alu_machined"))
    body.append(lathe("mglass", [(0.0, 0.0018), (0.0217, 0.0018), (0.0217, 0.0025), (0.0, 0.0025)], M=FH, segs=64,
                      mat="mirror"))

    body_ob = join(body, "clipon_%s_body" % tag)
    finish(body_ob, 35.0, 0.0007, 3, 30.0)

    # ---- bar end weight (turned, local Z along bar)
    be = lathe("clipon_%s_barend" % tag,
               [(0.0, 0.0), (0.0148, 0.0), (0.0160, 0.0008), (0.0172, 0.0030), (0.0175, 0.0060), (0.0175, 0.0084),
                (0.0167, 0.0090), (0.0167, 0.0100), (0.0175, 0.0106), (0.0175, 0.0170), (0.0170, 0.0205),
                (0.0155, 0.0228), (0.0130, 0.0242), (0.0068, 0.0245), (0.0062, 0.0240), (0.0062, 0.0205),
                (0.0, 0.0205)], segs=64, mat="alu_turned")
    be.matrix_basis = Mbe
    finish(be, 40.0, 0.0003, 2, 35.0)

    # ---- lever (brushed along local X = bar direction), pivot at origin, local Z = pivot axis
    beta = rad(4.0)
    fl = (f * math.cos(beta) + n * math.sin(beta)).normalized()
    zl = d.cross(fl)
    ML = basis(P, d, fl, zl)
    path = catmull([(0.000, 0.000), (0.012, 0.0068), (0.030, 0.0092), (0.060, 0.0084), (0.100, 0.0046),
                    (0.134, 0.0002), (0.157, -0.0040)], 6)
    blade = sweep_xy("blade", path, (0.0108, 0.0066), (0.0074, 0.0060), mat="alu_brushed")
    pb = cylz("lboss", 0.0086, -0.0039, 0.0039, segs=40, bevel=0.0008, mat="alu_brushed")
    tang = box("tang", (-0.0085, -0.0135, -0.0034), (0.0045, 0.0, 0.0034), bevel=0.0012, mat="alu_brushed")
    end = path[-1]
    ball = sphere("lball", 0.0056, M=T(end.x, end.y, 0.0), mat="alu_brushed")
    dial = cylz("dial", 0.0056, 0.0105, 0.0205, M=frame((0, -0.0088, 0), (1, 0, 0), (0, 1, 0)), segs=18,
                bevel=0.0004, mat="ano_orange")
    dcap = cylz("dialc", 0.0030, 0.0205, 0.0225, M=frame((0, -0.0088, 0), (1, 0, 0), (0, 1, 0)), segs=16,
                mat="alu_brushed")
    lever = join([blade, pb, tang, ball, dial, dcap], "clipon_%s_lever" % tag)
    lever.matrix_basis = ML
    finish(lever, 60.0, 0.0, 0)

    # ---- reservoir cup (glass) + fluid
    cup = lathe("cup", [(0.0, 0.0245), (0.0182, 0.0245), (0.0185, 0.0255), (0.0185, 0.0545), (0.0173, 0.0545),
                        (0.0173, 0.0265), (0.0, 0.0265)], segs=48, mat="glass", smooth=60.0)
    fluid = lathe("fluid", [(0.0, 0.02665), (0.01715, 0.02665), (0.01715, 0.0445), (0.0, 0.0445)], segs=48,
                  mat="amber_fluid", smooth=60.0)
    res = join([cup, fluid], "clipon_%s_reservoir" % tag)
    res.matrix_basis = FR
    geo.set_smooth(res, 60.0)

    lines_ob = join(lines, "clipon_%s_lines" % tag)
    geo.set_smooth(lines_ob, 60.0)
    objs = [body_ob, be, lever, res, grip, lines_ob]
    for o in objs:
        geo.parent(o, root, keep_world=False)
    return objs


# ============================================================================ DETAIL PASS 2 helpers
def line(name, pts, r, mat="braid", sub=8, segs=16):
    P = catmull([Vector(p) for p in pts], sub)
    ob = geo.tube(name, [tuple(p) for p in P], r, segs=segs, col=_col())
    return setmat(ob, mat)


def hose_end(pos, axis, nut="steel_polished", thread=True):
    """Crimped hose end: ferrule behind pos (hose arrives travelling along axis), hex nut (+thread) ahead."""
    F = frame(Vector(pos), Vector(axis).normalized())
    out = [cylz("ferr", 0.0049, -0.012, 0.0, M=F, segs=24, bevel=0.0005, mat="steel_polished"),
           cylz("nut", 0.0062, 0.0, 0.008, M=F, segs=6, bevel=0.0006, mat=nut)]
    if thread:
        out.append(cylz("thr", 0.0040, 0.008, 0.0105, M=F, segs=20, mat="zinc"))
    return out


def tie(c, axis, r):
    F = frame(Vector(c), Vector(axis).normalized())
    return [lathe("tie", [(r, -0.0012), (r + 0.0009, -0.0012), (r + 0.0009, 0.0012), (r, 0.0012)], M=F, segs=20,
                  closed=True, mat="plastic_black"),
            box("tieh", (r - 0.0002, -0.0016, -0.0017), (r + 0.0030, 0.0016, 0.0017), M=F, mat="plastic_black")]


def banjo_stack(origin, axis, x_hint, start=0.0, eyes=1):
    """Copper crush washers + eye(s) + hex bolt along axis from `start`.  Returns (parts, eye centre offsets)."""
    F = frame(Vector(origin), Vector(axis), x_hint)
    parts, centres = [], []
    z = start
    seq = [("copper", 0.0013, 0.0078, 28)]
    for _ in range(eyes):
        seq += [("eye", 0.0070, 0.0072, 28), ("copper", 0.0013, 0.0078, 28)]
    seq += [("hex", 0.0055, 0.0064, 6)]
    for kind, h, r, sg in seq:
        mt = {"copper": "copper", "eye": "steel_polished", "hex": "steel_polished"}[kind]
        parts.append(cylz("bj", r, z, z + h, M=F, segs=sg, bevel=0.0005 if h > 0.003 else 0.0, mat=mt))
        if kind == "eye":
            centres.append(z + h / 2)
        z += h
    return parts, centres


def twisted_loop(c, d, f, n, r, name="wire"):
    """Safety-wire pair twisted around a bar (axis d) at radius r."""
    out = []
    for ph in (0.0, PI):
        pts = []
        for k in range(97):
            th = 2 * PI * k / 96
            rd = f * math.cos(th) + n * math.sin(th)
            pts.append(tuple(c + rd * (r + 0.0003 * math.sin(14 * th + ph)) + d * (0.00055 * math.cos(14 * th + ph))))
        ob = geo.tube(name, pts, 0.00042, segs=8, col=_col())
        out.append(setmat(ob, "steel_polished"))
    # pigtail
    base = c - n * r
    pts = [tuple(base + d * (0.0005 * math.sin(k * 1.4)) - n * (0.0012 * k)) for k in range(6)]
    out.append(setmat(geo.tube(name, pts, 0.0005, segs=8, col=_col()), "steel_polished"))
    return out


def annular_sector(r0, r1, a0, a1, n=24):
    pts = [(r1 * math.cos(a0 + (a1 - a0) * k / n), r1 * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]
    pts += [(r0 * math.cos(a0 + (a1 - a0) * k / n), r0 * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n, -1, -1)]
    return pts


# ---------------------------------------------------------------------------- steer details
def steer_details():
    """Brake/clutch lines + fittings, splitter, headlight loom, turn signals, fork pads (all in F)."""
    fit, lines, sig, pads = [], [], [], []
    for s in (1, -1):
        # receiving fitting right under the clip-on line end (clip-on end nut bottom at F.z ~ +0.011)
        fit += hose_end((TX + 0.045, s * 0.093, -0.002), (0, 0, 1), "ano_blue" if s > 0 else "ano_red", thread=False)
        top = [(0.077, s * 0.093, -0.014), (0.077, s * 0.093, -0.035), (0.0785, s * 0.077, -0.075)]
        if s < 0:      # brake: down to the splitter on the lower clamp
            pts = top + [(0.074, s * 0.045, -0.128), (0.066, s * 0.012, -0.152), (0.064, s * 0.008, -0.164)]
            fit += hose_end((0.064, s * 0.008, -0.176), (0, 0, -1), thread=False)
        else:          # clutch: down through a guide loop to a bulkhead fitting under the clamp
            pts = top + [(0.0815, 0.066, -0.140), (0.0805, 0.062, -0.200), (0.079, 0.062, -0.226)]
            fit += hose_end((0.079, 0.062, -0.238), (0, 0, -1))
            fit.append(lathe("loop", [(0.0050, -0.0011), (0.0062, -0.0011), (0.0062, 0.0011), (0.0050, 0.0011)],
                             M=T(0.0805, 0.062, -0.196), segs=24, closed=True, mat="steel_polished"))
            fit.append(box("looptab", (0.0700, 0.0585, -0.1995), (0.0762, 0.0655, -0.1925), bevel=0.0006,
                           mat="steel_polished"))
            fit.append(bolt("loopb", 0.003, (0.0762, 0.062, -0.196), (1, 0, 0), kind="button", mat="steel_polished"))
        lines.append(line("sline", pts, 0.0035))
    # splitter block on the lower clamp front face
    fit.append(box("splitter", (0.052, -0.018, -0.212), (0.074, 0.018, -0.184), bevel=0.0016, segs=2,
                   mat="alu_machined"))
    for yy in (-0.0085, 0.0085):
        fit.append(bolt("spb", 0.0045, (0.074, yy, -0.192), (1, 0, 0), washer=True))
    for s in (1, -1):
        bj, cen = banjo_stack((0.063, s * 0.018, -0.203), (0, s, 0), (1, 0, 0), 0.0, 1)
        fit += bj
        ec = Vector((0.063, s * (0.018 + cen[0]), -0.203))
        fit.append(cylz("bjn", 0.0042, 0.0, 0.012, M=frame(ec, (0, 0, -1)), segs=20, mat="steel_polished"))
        fit.append(cylz("bjf", 0.0049, 0.012, 0.024, M=frame(ec, (0, 0, -1)), segs=24, bevel=0.0005,
                        mat="steel_polished"))
        st = ec + Vector((0, 0, -0.024))
        lines.append(line("short", [st, st + Vector((0.0, 0.0, -0.006)), (0.066, s * 0.040, -0.238),
                                    (0.069, s * 0.075, -0.2405), (0.070, s * 0.096, -0.240), (0.070, s * 0.108, -0.240)],
                          0.0035))
        fit += hose_end((0.070, s * 0.120, -0.240), (0, s, 0), thread=False)
    # steering damper: lower-clamp tab -> damper body -> clamp ring on the right outer tube
    zd = -0.2285
    fit.append(box("sdtab", (0.026, -0.016, -0.2215), (0.044, 0.000, -0.2150), bevel=0.0010, mat="alu_machined"))
    fit.append(cylz("sdeye", 0.0062, -0.016, -0.004, M=T(0.035, 0, zd) @ XZ, segs=28, bevel=0.0006,
                    mat="alu_machined"))
    fit.append(lathe("sdbody", [(0.0, 0.0), (0.0078, 0.0), (0.0086, 0.0012), (0.0086, 0.034), (0.0078, 0.0352),
                                (0.0, 0.0352)], M=frame((0.035, -0.019, zd), (0, -1, 0)), segs=36, mat="ano_black"))
    fit.append(lathe("sdcap", [(0.0, 0.0), (0.0089, 0.0), (0.0089, 0.0045), (0.0080, 0.0055), (0.0, 0.0055)],
                     M=frame((0.035, -0.0175, zd), (0, -1, 0)), segs=36, mat="alu_machined"))
    fit.append(cylz("sdknob", 0.0060, 0.0, 0.004, M=frame((0.035, -0.0175, zd + 0.0086), (0, 0, 1)), segs=18,
                    bevel=0.0004, mat="ano_gold"))
    fit.append(cylz("sdrod", 0.0030, 0.0352, 0.054, M=frame((0.035, -0.019, zd), (0, -1, 0)), segs=20,
                    mat="chrome"))
    fit.append(lathe("sdring", [(0.0272, -0.005), (0.0335, -0.005), (0.0340, -0.004), (0.0340, 0.004),
                                (0.0335, 0.005), (0.0272, 0.005)], M=T(TX, -TY, zd), segs=64, closed=True,
                     mat="alu_machined"))
    fit.append(box("sdlug", (0.029, -0.078, zd - 0.005), (0.041, -0.069, zd + 0.005), bevel=0.0009,
                   mat="alu_machined"))
    fit.append(bolt("sdb", 0.004, (0.035, -0.069, zd), (0, 1, 0), washer=True))
    fit.append(bolt("sdb2", 0.004, (0.035, -0.004, zd), (0, 1, 0), washer=True))
    # headlight loom -> connector block on the lower clamp
    loom = [(0.0768, 0.0, -0.0405), (0.0715, 0.0, -0.0435), (0.0665, 0.004, -0.062), (0.062, 0.010, -0.105),
            (0.053, 0.014, -0.138), (0.048, 0.016, -0.152), (0.048, 0.016, -0.159)]
    lines.append(line("loom", loom, 0.0052, mat="rubber", segs=20))
    for c, a in ((Vector((0.0638, 0.0055, -0.080)), Vector((-0.08, 0.13, -1.0))),
                 (Vector((0.0565, 0.0125, -0.127)), Vector((-0.25, 0.1, -1.0)))):
        fit += tie(c, a, 0.0054)
    fit.append(box("conn", (0.038, 0.004, -0.1752), (0.058, 0.028, -0.157), bevel=0.0012, mat="plastic_black"))
    fit.append(box("connl", (0.0405, 0.0075, -0.157), (0.0465, 0.0245, -0.1545), bevel=0.0006, mat="plastic_black"))
    fit.append(box("connr", (0.0575, 0.008, -0.172), (0.0595, 0.024, -0.160), bevel=0.0005, mat="rubber"))
    # turn-signal pods on stalks from the headlight ears
    a = Vector((math.cos(RAKE), 0, -math.sin(RAKE)))
    zc, zb, xb = -0.110, (HEAD_C + a * (-0.030)).z, (HEAD_C + a * (-0.030)).x
    zs = zc + (zb - zc) * (0.160 - 0.048) / (xb - 0.048)
    for s in (1, -1):
        y0, y1 = sorted((s * 0.1015, s * 0.1215))
        sig.append(cylz("stalk", 0.0036, y0, y1, M=T(0.160, 0, zs) @ XZ, segs=20, bevel=0.0003, mat="alu_machined"))
        n0, n1 = sorted((s * 0.0990, s * 0.0940))
        sig.append(cylz("stalkn", 0.0050, n0, n1, M=T(0.160, 0, zs) @ XZ, segs=6, bevel=0.0003, mat="steel_dark"))
        Fp = frame((0.160, s * 0.1268, zs), a, (0, 0, 1))
        sig.append(lathe("pod", [(0.0, -0.011), (0.0055, -0.011), (0.0072, -0.0085), (0.0076, -0.004),
                                 (0.0076, 0.0060), (0.0069, 0.0080), (0.0, 0.0080)], M=Fp, segs=32, mat="ano_black"))
        sig.append(lathe("podr", [(0.0060, 0.0072), (0.0071, 0.0072), (0.0073, 0.0080), (0.0073, 0.0090),
                                  (0.0060, 0.0090)], M=Fp, segs=32, closed=True, mat="chrome"))
        sig.append(lathe("podl", [(0.0, 0.0128), (0.0035, 0.0124), (0.0055, 0.0110), (0.0062, 0.0090),
                                  (0.0062, 0.0076), (0.0, 0.0076)], M=Fp, segs=32, mat="amber_fluid"))
    # abstract machined pads on the outer tubes (outboard faces)
    for s in (1, -1):
        for (a0, a1, z0, z1) in ((50, 130, -0.345, -0.290), (60, 120, -0.362, -0.356)):
            aa0, aa1 = sorted((s * rad(a0), s * rad(a1)))
            pads.append(prism("pad", annular_sector(0.02685, 0.0276, aa0, aa1, 24), z0, z1, M=T(TX, s * TY, 0),
                              mat="alu_machined"))
    return fit, lines, sig, pads


# ---------------------------------------------------------------------------- lower details
def lower_details(cal_ends):
    """Caliper feed lines up the legs, crossover, carbon guards with clips, wheel-speed sensor."""
    fit, lines, guards = [], [], []
    for s in (1, -1):
        # carbon guard in front/outboard of the stanchion
        a0, a1 = sorted((s * rad(-8.0), s * rad(100.0)))
        guards.append(prism("guard", annular_sector(0.0336, 0.0362, a0, a1, 40), -0.652, -0.470, M=T(TX, s * TY, 0),
                            mat="carbon"))
        for ang in (8.0, 88.0):
            ca = s * rad(ang)
            Fm = basis((TX + 0.031 * math.cos(ca), s * TY + 0.031 * math.sin(ca), -0.646),
                       (math.cos(ca), math.sin(ca), 0), (-math.sin(ca), math.cos(ca), 0), (0, 0, 1))
            fit.append(box("gtab", (-0.0035, -0.006, -0.006), (0.0040, 0.006, 0.006), M=Fm, bevel=0.0008,
                           mat="alu_machined"))
            fit.append(bolt("gtb", 0.0035, Fm @ Vector((0.0053, 0, 0)), (math.cos(ca), math.sin(ca), 0), kind="button",
                            mat="steel_polished"))
        # feed line from the caliper banjo up to the fitting under the lower clamp
        q0, dq = cal_ends[s][0]
        ca = s * rad(103.0)
        cl = Vector((TX + 0.047 * math.cos(ca), s * TY + 0.047 * math.sin(ca), 0))
        pts = [q0, q0 + dq * 0.008, (-0.012, s * 0.118, -0.566), (0.004, s * 0.1445, -0.556),
               (cl.x, cl.y, -0.540), (cl.x, cl.y, -0.488), (0.030, s * 0.160, -0.420), (0.054, s * 0.162, -0.330),
               (0.067, s * 0.160, -0.272), (0.070, s * 0.1585, -0.251), (0.070, s * 0.1540, -0.2405),
               (0.070, s * 0.1495, -0.240)]
        lines.append(line("feed", pts, 0.0035))
        fit += hose_end((0.070, s * 0.1375, -0.240), (0, -s, 0), thread=False)
        # guide clips on the guard edge (brake line + sensor wire on the left)
        for zc in (-0.540, -0.488):
            Fm = basis((TX + 0.0405 * math.cos(ca), s * TY + 0.0405 * math.sin(ca), zc),
                       (math.cos(ca), math.sin(ca), 0), (-math.sin(ca), math.cos(ca), 0), (0, 0, 1))
            fit.append(box("clip", (-0.0060, -0.0042, -0.0045), (0.0120 if s > 0 else 0.0035, 0.0042, 0.0045), M=Fm,
                           bevel=0.0009, mat="alu_machined"))
            fit.append(lathe("grom", [(0.0036, -0.0040), (0.0058, -0.0040), (0.0058, 0.0040), (0.0036, 0.0040)],
                             M=T(cl.x, cl.y, zc), segs=24, closed=True, mat="rubber"))
            if s > 0:
                cw = Vector((TX + 0.054 * math.cos(ca), s * TY + 0.054 * math.sin(ca), zc))
                fit.append(lathe("grom", [(0.0023, -0.0035), (0.0040, -0.0035), (0.0040, 0.0035), (0.0023, 0.0035)],
                                 M=T(cw.x, cw.y, zc), segs=20, closed=True, mat="rubber"))
    # crossover over the fender between the second banjo eyes
    qL, dL = cal_ends[1][1]
    qR, dR = cal_ends[-1][1]
    half = [(-0.040, 0.112, -0.530), (-0.045, 0.092, -0.470), (-0.037, 0.080, -0.400), (-0.032, 0.062, -0.378)]
    pts = [qL, qL + dL * 0.008] + half + [(-0.029, 0.0, -0.3715)] + \
          [(x, -y, z) for (x, y, z) in reversed(half)] + [qR + dR * 0.008, qR]
    lines.append(line("cross", pts, 0.0035, sub=10))
    # wheel-speed sensor on the left foot web + wire up the leg
    s = 1
    fit.append(cylz("wss", 0.0062, 0.0855, 0.1160, M=T(-0.035, 0, -0.695) @ XZ, segs=28, bevel=0.0007,
                    mat="plastic_black"))
    fit.append(box("wssear", (-0.050, 0.1032, -0.6995), (-0.035, 0.1105, -0.6905), bevel=0.0012, mat="plastic_black"))
    fit.append(bolt("wssb", 0.004, (-0.0455, 0.1105, -0.695), (0, 1, 0), mat="steel_dark"))
    fit.append(cylz("wssg", 0.0040, 0.1160, 0.1215, M=T(-0.035, 0, -0.695) @ XZ, segs=20, mat="rubber"))
    ca = rad(103.0)
    cw = Vector((TX + 0.054 * math.cos(ca), TY + 0.054 * math.sin(ca), 0))
    wpts = [(-0.035, 0.1205, -0.695), (-0.034, 0.128, -0.684), (-0.012, 0.152, -0.600), (cw.x, cw.y, -0.540),
            (cw.x, cw.y, -0.488), (0.031, 0.1675, -0.420), (0.049, 0.1690, -0.350), (0.056, 0.1665, -0.315)]
    lines.append(line("wsswire", wpts, 0.0021, mat="rubber", segs=12))
    dw = (Vector(wpts[-1]) - Vector(wpts[-2])).normalized()
    fit.append(cylz("wssc", 0.0042, 0.0, 0.014, M=frame(Vector(wpts[-1]), dw), segs=6, bevel=0.0006,
                    mat="plastic_black"))
    return fit, lines, guards


# ============================================================================ BUILD
def _root(name, loc_f, col):
    e = bpy.data.objects.new(name, None)
    e.empty_display_size = 0.06
    e.empty_display_type = "ARROWS"
    col.objects.link(e)
    e.location = C.fork_to_world(loc_f)
    e.rotation_euler = (0.0, -RAKE, 0.0)
    return e


def _in_scene(col):
    def walk(c):
        if c == col:
            return True
        return any(walk(ch) for ch in c.children)
    return walk(bpy.context.scene.collection)


def build(col):
    """Booleans / joins are evaluated through the depsgraph, so `col` must be part of the active scene
    while building; if it is not, it is linked to the scene temporarily."""
    tmp_link = not _in_scene(col)
    if tmp_link:
        bpy.context.scene.collection.children.link(col)
    try:
        return _build(col)
    finally:
        if tmp_link:
            bpy.context.scene.collection.children.unlink(col)


def _build(col):
    _STATE["col"] = col
    roots = {
        "steer": _root("front_steer", (0, 0, 0), col),
        "lower": _root("front_lower", (0, 0, 0), col),
        "clipon_L": _root("front_clipon_L", (TX, TY, -0.055), col),
        "clipon_R": _root("front_clipon_R", (TX, -TY, -0.055), col),
    }

    # ------------------------------------------------------------ steer
    steer = []
    dark = []
    machined = []
    machined.append(triple_clamp("upper_clamp", -0.030, 0.0, True, 0.0165, [-0.015], dark))
    machined.append(triple_clamp("lower_clamp", -0.215, -0.175, False, 0.0162, [-0.2045, -0.1855], dark))
    for s in (1, -1):
        machined.append(head_bracket(s, dark, machined))
    machined.append(gauge_bracket())
    gfr, gn, gu = gauge_frames()
    cmid = Vector((0.100, 0.0, 0.048))
    for xx in (-0.0085, 0.0085):
        dark.append(bolt("gbb", 0.005, (0.042, xx, 0.0055), (0, 0, 1)))
    for s, G in gfr:
        dark.append(bolt("gstud", 0.005, G @ Vector((0, 0, -0.0362)), -gn, kind="hex", washer=True))
    clamps = join(machined, "fr_clamps")
    setmat(clamps, "alu_machined")
    finish(clamps, 30.0, 0.0009, 3, 30.0)
    steer.append(clamps)

    stem = stem_nut()
    finish(stem, 40.0, 0.0004, 2, 40.0)
    steer.append(stem)

    tube_me = None
    for s, tag in ((1, "L"), (-1, "R")):
        ot = lathe("fr_outer_" + tag, outer_tube_profile(), segs=72, mat="ano_titan", smooth=40.0)
        if tube_me is None:
            tube_me = ot.data
        else:
            old = ot.data
            ot.data = tube_me
            bpy.data.meshes.remove(old)
        ot.matrix_basis = T(TX, s * TY, 0)
        finish(ot, 40.0, 0.0004, 2, 40.0)
        steer.append(ot)
    cap_L = fork_cap("fr_forkcap_L")
    cap_L.matrix_basis = T(TX, TY, 0.012)
    finish(cap_L, 40.0, 0.0003, 2, 40.0)
    cap_R = geo.copy_obj(cap_L, "fr_forkcap_R", col=col, linked=True)
    cap_R.matrix_basis = T(TX, -TY, 0.012)
    steer += [cap_L, cap_R]
    for s in (1, -1):
        dark.append(dust_seal(s))

    # headlight
    H = T(HEAD_C) @ Matrix.Rotation(RAKE + PI / 2, 4, "Y")
    hparts, emit = headlight_parts()
    head = join(hparts, "fr_headlight")
    head.matrix_basis = H
    emit.matrix_basis = H
    mats.set_emit(emit, 1.0)
    steer += [head, emit]

    # gauges
    gparts = []
    for s, G in gfr:
        p, needle = gauge_parts(s, G)
        gparts += p
        steer.append(needle)
    G0 = basis(cmid, (0, -1, 0), gu, gn)
    gparts.append(prism("wlp", rrect(0.0, 0.0, 0.0058, 0.024, 0.0035), -0.004, 0.0015, M=G0, mat="ano_black"))
    gparts.append(prism("wls", rrect(0.0, -0.004, 0.0040, 0.012, 0.003), -0.0325, -0.0035, M=G0, mat="ano_black"))
    for uu, mt in ((-0.0165, "ano_blue"), (-0.0055, "amber_fluid"), (0.0055, "glass_red"), (0.0165, "amber_fluid")):
        Fl = G0 @ T(0.0, uu, 0.0)
        gparts.append(lathe("wlr", [(0.0025, 0.0015), (0.0034, 0.0015), (0.0035, 0.0024), (0.0030, 0.0028),
                                    (0.0025, 0.0026)], M=Fl, segs=24, closed=True, mat="chrome"))
        gparts.append(lathe("wll", [(0.0, 0.0031), (0.0015, 0.0029), (0.0025, 0.0022), (0.0026, 0.0012),
                                    (0.0, 0.0012)], M=Fl, segs=24, mat=mt))
    gauges = join(gparts, "fr_gauges")
    geo.set_smooth(gauges, 40.0)
    steer.append(gauges)

    sfit, slines, ssig, spads = steer_details()
    o1 = join(sfit, "fr_steer_fittings")
    finish(o1, 35.0, 0.0004, 2, 30.0)
    o2 = join(slines, "fr_steer_lines")
    geo.set_smooth(o2, 60.0)
    o3 = join(ssig, "fr_turn_signals")
    finish(o3, 40.0, 0.0003, 2, 35.0)
    o4 = join(spads, "fr_fork_pads")
    finish(o4, 30.0, 0.0003, 2, 30.0)
    steer += [o1, o2, o3, o4]

    sd = join(dark, "fr_steer_hw")
    geo.set_smooth(sd, 40.0)
    steer.append(sd)
    for o in steer:
        geo.parent(o, roots["steer"], keep_world=False)

    # ------------------------------------------------------------ lower
    lower = []
    bright = [stanchion(1), stanchion(-1)] + axle_parts()
    feet = []
    cal = []
    cal_ends = {}
    for s in (1, -1):
        K = caliper_matrix(s)
        feet.append(fork_foot(s, K, bright))
        cal_ends[s] = caliper(s, K, cal)
    lfit, llines, lguards = lower_details(cal_ends)
    o5 = join(lfit, "fr_lower_fittings")
    finish(o5, 35.0, 0.0004, 2, 30.0)
    o6 = join(llines, "fr_lower_lines")
    geo.set_smooth(o6, 60.0)
    o7 = join(lguards, "fr_fork_guards")
    finish(o7, 30.0, 0.0007, 2, 30.0)
    lower += [o5, o6, o7]
    feet_ob = join(feet, "fr_fork_feet")
    setmat(feet_ob, "alu_machined")
    finish(feet_ob, 30.0, 0.0009, 3, 30.0)
    lower.append(feet_ob)
    cal_ob = join(cal, "fr_calipers")
    geo.set_smooth(cal_ob, 30.0)
    weight_bevel(cal_ob, "ano_black", 0.0034, 4, 30.0)
    finish(cal_ob, 30.0, 0.0008, 2, 30.0)
    lower.append(cal_ob)
    br = join(bright, "fr_lower_bright")
    finish(br, 40.0, 0.0003, 2, 40.0)
    lower.append(br)
    MF = T(WC) @ Matrix.Rotation(RAKE, 4, "Y")
    fparts = [fender_parts()]
    for s in (1, -1):
        fparts += fender_stays(s, MF.inverted())
    fen = join(fparts, "fr_fender")
    fen.matrix_basis = MF
    finish(fen, 45.0, 0.0005, 2, 40.0)
    lower.append(fen)
    for o in lower:
        geo.parent(o, roots["lower"], keep_world=False)

    # ------------------------------------------------------------ clip-ons
    clipon(1, roots["clipon_L"], "L")
    clipon(-1, roots["clipon_R"], "R")

    bpy.context.view_layer.update()
    return roots
