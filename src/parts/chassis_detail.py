"""Chassis detail pass: weld beads, gussets, bosses, rearsets, electrics, looms, swingarm / carriage / head detail.

build(col) -> dict of lists of mesh objects keyed by rig group:
  CH   static frame parts (final-pose world coordinates)
  CARR seat carriage parts (final-pose world coordinates)
  SAP  swingarm pivot block / bell crank (final-pose world coordinates)
  L1   inner swingarm link (link-local frame, hinge at origin, beam along -X, Z = hinge axis)
  L2   outer swingarm link (link-local frame, elbow at origin, tongue along -X)
  HEAD head carrier (fork-frame coordinates)
Small parts are joined per (group, material).  check(res) tests every vertex against the keep-out zones.
"""
import math
import bpy
import bmesh
from mathutils import Vector, Matrix

import common as C
import geo
import mats

COL = None
G = {}


def M(n):
    return mats.get(n)


# ----------------------------------------------------------------------------- low level helpers
def add(g, ob, mat):
    geo.set_mat(ob, M(mat))
    G.setdefault(g, {}).setdefault(mat, []).append(ob)
    return ob


def frame_z(loc, zdir, xhint=None):
    z = Vector(zdir).normalized()
    x = Vector(xhint) if xhint is not None else (Vector((1, 0, 0)) if abs(z.x) < 0.9 else Vector((0, 1, 0)))
    x = x - z * x.dot(z)
    if x.length < 1e-6:
        x = z.orthogonal()
    x.normalize()
    y = z.cross(x)
    m = Matrix((x, y, z)).transposed().to_4x4()
    m.translation = Vector(loc)
    return m


def frame_uv(O, U, V):
    U = Vector(U).normalized()
    V = Vector(V)
    V = (V - U * V.dot(U)).normalized()
    N = U.cross(V)
    m = Matrix((U, V, N)).transposed().to_4x4()
    m.translation = Vector(O)
    return m


def xf(ob, m):
    ob.data.transform(m)
    ob.data.update()
    return ob


def cyl(g, mat, r, h, loc, zdir=(0, 0, 1), segs=24, bevel=0.0, r2=None, xhint=None):
    o = geo.cyl("cd", r, h, segs, bevel=bevel, col=COL, r2=r2)
    return add(g, xf(o, frame_z(loc, zdir, xhint)), mat)


def box(g, mat, sx, sy, sz, loc, zdir=(0, 0, 1), xhint=None, bevel=0.0):
    o = geo.box("cd", sx, sy, sz, bevel=bevel, segs=2, col=COL)
    return add(g, xf(o, frame_z(loc, zdir, xhint)), mat)


def bolt(g, loc, n, d=0.005, kind="socket", mat="steel_dark", washer=False):
    o = geo.bolt("cd", d, kind=kind, col=COL, washer=washer)
    return add(g, xf(o, frame_z(loc, n)), mat)


def tube(g, mat, pts, r, segs=12, closed=False, radii=None, fillet=0.0):
    o = geo.tube("cd", pts, r, segs, fillet=fillet, closed=closed, col=COL, radii=radii)
    return add(g, o, mat)


def torus(g, mat, R, r, loc, zdir, segs=24, rsegs=8):
    o = geo.torus("cd", R, r, segs, rsegs, "Z", COL)
    return add(g, xf(o, frame_z(loc, zdir)), mat)


def lathe(g, mat, prof, loc, zdir, segs=24, closed=True, smooth=40.0):
    o = geo.lathe("cd", prof, segs, "Z", COL, closed=closed, smooth=smooth)
    return add(g, xf(o, frame_z(loc, zdir)), mat)


def prism(g, mat, poly, t, O, U, V, bevel=0.0006, bsegs=1):
    o = geo.prism("cd", poly, t, bevel=bevel, bsegs=bsegs, plane="XY", col=COL)
    return add(g, xf(o, frame_uv(O, U, V)), mat)


def rrect(cx, cy, L, W, r, n=5):
    r = min(r, L / 2 - 1e-5, W / 2 - 1e-5)
    pts = []
    for (qx, qy, a0) in ((cx + L / 2 - r, cy + W / 2 - r, 0.0), (cx - L / 2 + r, cy + W / 2 - r, 90.0),
                         (cx - L / 2 + r, cy - W / 2 + r, 180.0), (cx + L / 2 - r, cy - W / 2 + r, 270.0)):
        for k in range(n + 1):
            a = math.radians(a0 + 90.0 * k / n)
            pts.append((qx + r * math.cos(a), qy + r * math.sin(a)))
    return pts


def circle(cx, cy, r, n=24):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def stadium(p0, p1, w0, w1, n=8):
    """2D link outline between p0 and p1 with end radii w0, w1 (CCW)."""
    p0, p1 = Vector(p0), Vector(p1)
    d = (p1 - p0).normalized()
    a = math.atan2(d.y, d.x)
    pts = []
    for k in range(n + 1):
        t = a - math.pi / 2 + math.pi * k / n
        pts.append((p1.x + w1 * math.cos(t), p1.y + w1 * math.sin(t)))
    for k in range(n + 1):
        t = a + math.pi / 2 + math.pi * k / n
        pts.append((p0.x + w0 * math.cos(t), p0.y + w0 * math.sin(t)))
    return pts


def _ray(poly, c, ang):
    ox, oy = c
    dx, dy = math.cos(ang), math.sin(ang)
    best = None
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        s = ((ax - ox) * ey - (ay - oy) * ex) / den
        u = ((ax - ox) * dy - (ay - oy) * dx) / den
        if s > 0 and -1e-9 <= u <= 1 + 1e-9:
            if best is None or s < best:
                best = s
    return best if best is not None else 0.0


def ring(g, mat, outer, inner, c, t, O, U, V, n=48, ch=0.0003, out=None):
    """Plate between two star-shaped 2D outlines (about c), thickness t along +N (or toward `out`) from the plane."""
    flip = out is not None and frame_uv(O, U, V).to_3x3().col[2].dot(Vector(out)) < 0
    angs = set(2 * math.pi * k / n for k in range(n))
    for poly in (outer, inner):
        if len(poly) <= 8:
            for (px, py) in poly:
                angs.add(math.atan2(py - c[1], px - c[0]) % (2 * math.pi))
    angs = sorted(angs)
    bm = bmesh.new()
    rings = [[] for _ in range(6)]
    for a in angs:
        so = _ray(outer, c, a)
        si = _ray(inner, c, a)
        ca, sa = math.cos(a), math.sin(a)
        e = min(ch, (so - si) * 0.25)
        spec = ((so, 0.0), (so, t - e), (so - e, t), (si + e, t), (si, t - e), (si, 0.0))
        for j, (s, z) in enumerate(spec):
            rings[j].append(bm.verts.new((c[0] + s * ca, c[1] + s * sa, z)))
    m = len(angs)
    for j in range(6):
        a_, b_ = rings[j], rings[(j + 1) % 6]
        for k in range(m):
            k2 = (k + 1) % m
            try:
                bm.faces.new((a_[k], a_[k2], b_[k2], b_[k]))
            except ValueError:
                pass
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=1e-7)
    if flip:
        for v in bm.verts:
            v.co.z = -v.co.z
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    o = geo.obj_from_bm("cd_ring", bm, COL, smooth=40.0)
    return add(g, xf(o, frame_uv(O, U, V)), mat)


def pocket(g, O, U, V, c, L, W, r=0.004, rim=0.0022, t=0.0010, rim_mat="alu_machined", floor_mat=None, out=None):
    """Machined pocket look: bright chamfered rim frame (+ optional dark floor) sitting on a face (normal `out`)."""
    outer = rrect(c[0], c[1], L, W, r)
    inner = rrect(c[0], c[1], L - 2 * rim, W - 2 * rim, max(0.0008, r - rim))
    ring(g, rim_mat, outer, inner, c, t, O, U, V, n=56, out=out)
    if floor_mat:
        prism(g, floor_mat, inner, 0.0003, Vector(O) + Vector(out).normalized() * 0.00015, U, V, bevel=0.0)


# ----------------------------------------------------------------------------- frame helpers
TUBE_R = 0.0105


def weld_bead(g, P, Q, rt, R, Hh, n=48, rb=0.0019, ripples=13):
    """Rippled weld bead where a tube (P -> Q, radius rt) leaves a Y-axis lug (radius R, height Hh) centred at P."""
    P, Q = Vector(P), Vector(Q)
    d = (Q - P).normalized()
    e1 = d.cross(Vector((0, 1, 0)))
    if e1.length < 1e-6:
        e1 = Vector((1, 0, 0))
    e1.normalize()
    e2 = d.cross(e1).normalized()
    pts, radii = [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        w = (e1 * math.cos(a) + e2 * math.sin(a)) * (rt + 0.0005)
        ts = []
        A = d.x * d.x + d.z * d.z
        B = 2 * (d.x * w.x + d.z * w.z)
        Cc = w.x * w.x + w.z * w.z - R * R
        if A > 1e-9:
            disc = B * B - 4 * A * Cc
            if disc >= 0:
                ts.append((-B + math.sqrt(disc)) / (2 * A))
        if abs(d.y) > 1e-6:
            ts.append(((Hh / 2) * (1 if d.y > 0 else -1) - w.y) / d.y)
        ts = [t for t in ts if t > 0]
        t = min(ts) if ts else R
        pts.append(P + d * (t + 0.0007) + w)
        radii.append(1.0 + 0.22 * abs(math.sin(math.pi * k * ripples / n)))
    m = rb * 1.3
    for p in pts:
        for o in ((m, 0, 0), (-m, 0, 0), (0, 0, m), (0, 0, -m), (0, m, 0), (0, -m, 0)):
            if _zones_ch(p + Vector(o)):
                return None          # bead would poke into a keep-out zone (next to an existing lug): skip it
    tube(g, "ano_orange", pts, rb, segs=12, closed=True, radii=radii)


def gusset(g, P, Q1, Q2, L1=0.068, L2=0.068, t=0.003, start=0.013):
    P = Vector(P)
    d1 = (Vector(Q1) - P).normalized()
    d2 = (Vector(Q2) - P).normalized()
    Mf = frame_uv(P, d1, d2)
    inv = Mf.inverted()
    p1 = (inv @ (P + d1 * L1)).to_2d()
    p2 = (inv @ (P + d2 * L2)).to_2d()
    q1 = (inv @ (P + d1 * start)).to_2d()
    q2 = (inv @ (P + d2 * start)).to_2d()
    poly = [tuple(q1), tuple(p1), tuple(p2), tuple(q2)]
    # incircle of the triangle (0, p1, p2)
    a = (p1 - p2).length
    b = p2.length
    c = p1.length
    inc = (p1 * b + p2 * c) / (a + b + c)
    s = (a + b + c) / 2
    area = abs(p1.x * p2.y - p1.y * p2.x) / 2
    rin = area / s
    hr = rin - TUBE_R - 0.0032
    O = P - Mf.to_3x3() @ Vector((0, 0, t / 2))
    if hr > 0.0028:
        ring(g, "ano_orange", poly, circle(inc.x, inc.y, hr, 28), (inc.x, inc.y), t, O, d1, d2, n=56, ch=0.0005)
    else:
        prism(g, "ano_orange", poly, t, P, d1, d2, bevel=0.0005)


# ----------------------------------------------------------------------------- frame
def build_frame():
    g = "CH"
    for s in (1, -1):
        A = Vector((0.250, s * 0.118, 0.745))
        B = Vector((-0.02, s * 0.118, 0.745))
        Cc = Vector((-0.33, s * 0.125, 0.745))
        HB = Vector((0.10, s * 0.180, 0.615))
        RB = Vector((-0.19, s * 0.159, 0.52))
        big = (0.0165, 0.032)
        sm = (0.0185, 0.024)
        pairs = [(A, B, TUBE_R, big), (B, A, TUBE_R, big), (B, Cc, TUBE_R, big), (Cc, B, TUBE_R, big),
                 (A, HB, TUBE_R, big), (HB, A, TUBE_R, sm), (B, HB, TUBE_R, big), (HB, B, TUBE_R, sm),
                 (B, RB, TUBE_R, big), (RB, B, TUBE_R, sm), (Cc, RB, TUBE_R, big), (RB, Cc, TUBE_R, sm),
                 (HB, RB, 0.009, sm), (RB, HB, 0.009, sm),
                 (A, A - Vector((0, s, 0)), TUBE_R, big), (B, B - Vector((0, s, 0)), 0.009, big),
                 (Cc, Cc - Vector((0, s, 0)), TUBE_R, big)]
        if s > 0:
            pairs.append((Cc, Vector((-0.268, 0.186, 0.585)), TUBE_R, big))
            pairs.append((B, Vector((-0.04, 0.199, 0.735)), 0.008, big))
        for (P, Q, rt, (R, Hh)) in pairs:
            weld_bead(g, P, Q, rt, R, Hh)
        # gussets
        gusset(g, B, A, HB)
        gusset(g, B, Cc, RB, 0.07, 0.07)
        if s < 0:
            gusset(g, Cc, B, RB, 0.065, 0.065)
        # threaded bosses under the rails + button screws
        for x in (0.200, 0.075, -0.115, -0.255):
            y = s * (0.118 if x > -0.02 else 0.118 + 0.007 * (-0.02 - x) / 0.31)
            cyl(g, "ano_orange", 0.0052, 0.007, (x, y, 0.745 - TUBE_R - 0.0015), (0, 0, 1), 20, bevel=0.0008)
            bolt(g, (x, y, 0.745 - TUBE_R - 0.005), (0, 0, -1), 0.0042, "button", "steel_polished")
        # cable guide loops on the inside of the rails holding the loom
        for x in (0.165, 0.045, -0.150, -0.275):
            yr = s * (0.118 if x > -0.02 else 0.118 + 0.007 * (-0.02 - x) / 0.31)
            yl = s * 0.096
            torus(g, "steel_polished", 0.0058, 0.0011, (x, yl, 0.728), (1, 0, 0), 20, 6)
            tube(g, "steel_polished", [(x, yl + s * 0.0058 * 0.7, 0.7318), (x, yr - s * 0.0095, 0.738)], 0.0011, 8)
        # tube end plugs (tower cross tube ends on the tower faces)
        cyl(g, "alu_machined", 0.0115, 0.003, (0.232, s * 0.1345, 0.715), (0, s, 0), 32, bevel=0.0008)
        bolt(g, (0.232, s * 0.136, 0.715), (0, s, 0), 0.0055, "socket", "steel_polished")
        # tower plate: bolt circle around the big bore, diamond-cut edge strip, bolts
        for ang in (-30, 30, 75):
            a = math.radians(ang)
            bolt(g, (0.262 + 0.037 * math.cos(a), s * 0.133, 0.775 + 0.037 * math.sin(a)), (0, s, 0), 0.0050,
                 "socket", "steel_polished", washer=True)
        e0, e1 = Vector((0.2655, 0.8605)), Vector((0.287, 0.8885))
        dd = (e1 - e0).normalized()
        nn = Vector((dd.y, -dd.x))
        a0 = e0 + nn * 0.0035
        a1 = e1 + nn * 0.0035
        strip = [tuple(a0 + nn * 0.0016), tuple(a1 + nn * 0.0016), tuple(a1 - nn * 0.0016), tuple(a0 - nn * 0.0016)]
        prism(g, "alu_machined", strip, 0.0008, (0, s * 0.1334, 0), (1, 0, 0), (0, 0, 1), bevel=0.0003)
        bolt(g, (0.246, s * 0.133, 0.690), (0, s, 0), 0.0050, "socket", "steel_polished", washer=True)
        bolt(g, (0.205, s * 0.133, 0.722), (0, s, 0), 0.0050, "socket", "steel_polished", washer=True)
        # engine mounts: spacers, through bolts / nuts
        for (p, ly0, ly1) in (((0.10, 0.615), 0.160, 0.168), ((-0.19, 0.52), 0.139, 0.147)):
            cyl(g, "alu_machined", 0.0115, ly1 - ly0, (p[0], s * (ly0 + ly1) / 2, p[1]), (0, s, 0), 32, bevel=0.0007)
            for k in range(3):
                torus(g, "alu_machined", 0.0116, 0.0004, (p[0], s * (ly0 + 0.002 + 0.002 * k), p[1]), (0, 1, 0), 32, 4)
        if s < 0:
            for (p, yf) in (((0.10, 0.615), 0.192), ((-0.19, 0.52), 0.171)):
                bolt(g, (p[0], -yf, p[1]), (0, -1, 0), 0.0085, "hex", "steel_polished", washer=True)
                cyl(g, "steel_polished", 0.0042, 0.006, (p[0], -(yf + 0.010), p[1]), (0, -1, 0), 16, bevel=0.0006)
        bolt(g, (0.13, s * 0.143, 0.33), (0, s, 0), 0.0085, "hex", "steel_polished", washer=True)
        cyl(g, "steel_polished", 0.0042, 0.006, (0.13, s * 0.1555, 0.33), (0, s, 0), 16, bevel=0.0006)
    # blank riveted ID plate on the left tower
    box(g, "alu_brushed", 0.030, 0.0012, 0.019, (0.2075, 0.1336, 0.750), bevel=0.0005)
    for (dx, dz) in ((-0.0115, -0.0062), (0.0115, -0.0062), (-0.0115, 0.0062), (0.0115, 0.0062)):
        lathe(g, "steel_polished", [(0.0, 0.0012), (0.0012, 0.0009), (0.0016, 0.0), (0.0, 0.0)],
              (0.2075 + dx, 0.1342, 0.750 + dz), (0, 1, 0), 12)
    # pivot plate: bolt circle around the pivot cap + pocket
    for ang in (-30, 30, 90, 150, 210, 270):
        a = math.radians(ang)
        bolt(g, (C.PIVOT[0] + 0.032 * math.cos(a), 0.198, C.PIVOT[2] + 0.032 * math.sin(a)), (0, 1, 0), 0.0045,
             "socket", "steel_polished")
    pocket(g, (0, 0.198, 0), (1, 0, 0), (0, 0, 1), (-0.299, 0.505), 0.018, 0.060, 0.0055, out=(0, 1, 0))
    build_rearsets(g)
    build_electrics(g)


# ----------------------------------------------------------------------------- rearsets
def build_rearsets(g):
    PX, PZ = -0.238, 0.374
    XZ = ((1, 0, 0), (0, 0, 1))
    heel = [(-0.248, 0.350), (-0.222, 0.340), (-0.190, 0.370), (-0.186, 0.428), (-0.205, 0.445),
            (-0.240, 0.432), (-0.248, 0.420)]
    for s in (1, -1):
        # carbon heel plate on the hanger with a lightening bore, chamfered
        ring(g, "carbon", heel, circle(-0.210, 0.398, 0.0105, 28), (-0.210, 0.398), 0.0026,
             (0, s * 0.2192, 0), XZ[0], XZ[1], n=56, ch=0.0004, out=(0, s, 0))
        ring(g, "alu_machined", circle(-0.210, 0.398, 0.0125, 28), circle(-0.210, 0.398, 0.0105, 28),
             (-0.210, 0.398), 0.0006, (0, s * 0.2218, 0), XZ[0], XZ[1], n=28, ch=0.0002, out=(0, s, 0))
        for (bx, bz) in ((-0.196, 0.414), (-0.2415, 0.405)):
            bolt(g, (bx, s * 0.2218, bz), (0, s, 0), 0.0040, "socket", "steel_polished")
        # lever plate (alu) pivoting on the peg (right one sits outboard of y = -0.23 to clear the fold slab)
        yl0, yl1 = (0.2232, 0.2287) if s > 0 else (0.2302, 0.2357)
        ym = s * (yl0 + yl1) / 2
        toe = (-0.140, 0.356)
        prism(g, "alu_machined", stadium((PX, PZ), toe, 0.0075, 0.0055), yl1 - yl0, (0, ym, 0), XZ[0], XZ[1],
              bevel=0.0006)
        cyl(g, "alu_machined", 0.0145, 0.0070 if s > 0 else 0.0054, (PX, ym, PZ), (0, 1, 0), 36, bevel=0.0008)
        cyl(g, "steel_dark", 0.0042, 0.0016, (PX, s * (yl1 + 0.0009), PZ), (0, 1, 0), 6)
        # toe peg with knurl rings
        cyl(g, "alu_machined", 0.0068, 0.036, (toe[0], s * (yl1 + 0.018), toe[1]), (0, 1, 0), 24, bevel=0.001)
        for k in range(3):
            torus(g, "ano_black", 0.0068, 0.0011, (toe[0], s * (yl1 + 0.012 + 0.008 * k), toe[1]), (0, 1, 0), 24, 6)
        cyl(g, "steel_polished", 0.0045, 0.003, (toe[0], s * (yl1 + 0.0375), toe[1]), (0, 1, 0), 6)
        if s > 0:
            # shift lever: lower arm + linkage rod with rose joints to the gear shaft arm
            arm_end = (-0.232, 0.336)
            prism(g, "alu_machined", stadium((PX, PZ), arm_end, 0.0075, 0.0058), yl1 - yl0, (0, ym, 0), XZ[0],
                  XZ[1], bevel=0.0006)
            P1 = Vector((arm_end[0], 0.2335, arm_end[1]))
            P2 = Vector((-0.060, 0.2015, 0.330))
            rose_rod(g, P1, P2)
            # gear shaft stub + arm on the engine side
            cyl(g, "steel_polished", 0.0072, 0.016, (-0.046, 0.192, 0.352), (0, 1, 0), 24, bevel=0.0008)
            prism(g, "ano_black", stadium((-0.046, 0.352), (-0.060, 0.330), 0.0095, 0.0065), 0.004,
                  (0, 0.198, 0), XZ[0], XZ[1], bevel=0.0006)
            bolt(g, (-0.052, 0.200, 0.342), (0, 1, 0), 0.0035, "socket", "steel_dark")
        else:
            # brake pedal clevis arm, horizontal master cylinder, reservoir, braided line
            arm_end = (-0.229, 0.400)
            prism(g, "alu_machined", stadium((PX, PZ), arm_end, 0.0075, 0.0055), yl1 - yl0, (0, ym, 0), XZ[0],
                  XZ[1], bevel=0.0006)
            mcy = -0.240
            cyl(g, "alu_machined", 0.0078, 0.040, (-0.195, mcy, 0.400), (1, 0, 0), 28, bevel=0.0012)
            for x in (-0.205, -0.192, -0.179):
                torus(g, "alu_machined", 0.0079, 0.0006, (x, mcy, 0.400), (1, 0, 0), 28, 4)
            cyl(g, "ano_black", 0.0062, 0.004, (-0.2165, mcy, 0.400), (1, 0, 0), 24, bevel=0.0006)
            cyl(g, "steel_polished", 0.0022, 0.016, (-0.222, -0.2397, 0.400), (1, 0, 0.0), 12)
            box(g, "alu_machined", 0.008, 0.008, 0.006, (-0.229, -0.2397, 0.400), bevel=0.0008)
            box(g, "alu_machined", 0.010, 0.0115, 0.012, (-0.184, -0.2275, 0.400), bevel=0.0010)
            bolt(g, (-0.184, -0.2333, 0.4065), (0, 0, 1), 0.0030, "socket", "steel_dark")
            # banjo outlet
            cyl(g, "steel_polished", 0.0050, 0.006, (-0.205, mcy, 0.3905), (0, 0, 1), 6)
            # reservoir
            rx, ry = -0.140, -0.236
            cyl(g, "glass", 0.0092, 0.024, (rx, ry, 0.415), (0, 0, 1), 32)
            cyl(g, "amber_fluid", 0.0080, 0.016, (rx, ry, 0.4115), (0, 0, 1), 32)
            cyl(g, "ano_black", 0.0098, 0.005, (rx, ry, 0.4295), (0, 0, 1), 32, bevel=0.0009)
            cyl(g, "ano_black", 0.0094, 0.003, (rx, ry, 0.4015), (0, 0, 1), 32, bevel=0.0006)
            box(g, "alu_machined", 0.050, 0.0028, 0.008, (-0.168, -0.2232, 0.413), bevel=0.0006)
            box(g, "alu_machined", 0.012, 0.0028, 0.008, (-0.146, -0.2302, 0.413), (0, 0, 1), (0.6, -0.8, 0),
                bevel=0.0006)
            tube(g, "rubber", [(rx, ry, 0.400), (rx - 0.004, ry, 0.392), (-0.158, -0.239, 0.391), (-0.168, -0.240, 0.396),
                               (-0.1745, mcy, 0.400)], 0.0024, 12, fillet=0.004)
            torus(g, "zinc", 0.0028, 0.0007, (-0.150, -0.2385, 0.3915), (1, 0, 0.1), 16, 4)
            # braided hose back to the rear brake line
            hp = [(-0.205, mcy, 0.388), (-0.207, -0.240, 0.372), (-0.214, -0.236, 0.348), (-0.230, -0.223, 0.337),
                  (-0.240, -0.209, 0.346), (-0.243, -0.2015, 0.370), (-0.2435, -0.2005, 0.392)]
            tube(g, "braid", hp, 0.0036, 12, fillet=0.008)
            cyl(g, "steel_polished", 0.0046, 0.010, (-0.2435, -0.2005, 0.397), (0, 0, 1), 6, bevel=0.0005)
            cyl(g, "steel_polished", 0.0041, 0.008, (-0.205, mcy, 0.384), (0, 0, 1), 6, bevel=0.0005)


def rose_rod(g, P1, P2):
    d = (P2 - P1).normalized()
    for (P, sg) in ((P1, 1), (P2, -1)):
        # rod end eye (axis Y), ball, shank, jam nut
        cyl(g, "steel_polished", 0.0068, 0.0052, P, (0, 1, 0), 28, bevel=0.0008)
        lathe(g, "chrome", [(0.0, -0.0036), (0.0034, -0.0016), (0.0040, 0.0), (0.0034, 0.0016), (0.0, 0.0036)],
              P, (0, 1, 0), 16)
        cyl(g, "steel_polished", 0.0044, 0.014, P + d * sg * 0.0125, d * sg, 16, bevel=0.0006)
        cyl(g, "ano_blue", 0.0050, 0.0042, P + d * sg * 0.0215, d * sg, 6, bevel=0.0005)
        bolt(g, P + Vector((0, 0.0026, 0)), (0, 1, 0), 0.0034, "socket", "steel_dark")
    cyl(g, "steel_polished", 0.0032, (P2 - P1).length - 0.047, (P1 + P2) / 2, d, 16)


# ----------------------------------------------------------------------------- electrics / plumbing
def loom(g, pts, r=0.0046, ties=(), fillet=0.012):
    tube(g, "plastic_black", pts, r, 14, fillet=fillet)
    # thin companion wire spiralling loosely around the loom
    P = [Vector(p) for p in pts]
    for (u, rr) in ties:
        i = min(int(u * (len(P) - 1)), len(P) - 2)
        f = u * (len(P) - 1) - i
        c = P[i].lerp(P[i + 1], f)
        d = (P[i + 1] - P[i]).normalized()
        torus(g, "plastic_black", r + 0.0009, 0.0009, c, d, 20, 6)
        side = d.orthogonal().normalized()
        box(g, "plastic_black", 0.004, 0.0035, 0.003, c + side * (r + 0.0015), side, d)


def build_electrics(g):
    for s in (1, -1):
        y = s * 0.096
        pts = [(0.205, y, 0.728), (0.10, y, 0.728), (-0.02, y, 0.728), (-0.17, y, 0.728), (-0.300, s * 0.098, 0.728)]
        loom(g, pts, 0.0046, ties=[(0.12, 0), (0.37, 0), (0.62, 0), (0.88, 0)])
        tube(g, "copper", [(p[0], p[1] + s * 0.0, p[2] + 0.0052) for p in pts[:4]], 0.0011, 8)
        # front drop of the loom into the coil pack
        tube(g, "plastic_black", [(0.203, y, 0.728), (0.200, s * 0.085, 0.718), (0.185, s * 0.068, 0.705),
                                  (0.172, s * 0.058, 0.697)], 0.0034, 12, fillet=0.008)
        # connector blocks on the loom
        box(g, "plastic_black", 0.020, 0.0105, 0.0105, (0.080, y, 0.728), bevel=0.0012)
        box(g, "zinc", 0.003, 0.0108, 0.0108, (0.089, y, 0.728), bevel=0.0004)
        box(g, "plastic_black", 0.016, 0.0095, 0.0095, (-0.215, s * 0.0965, 0.728), bevel=0.0012)
        # drop along trellis tube tr3 (outer side)
        B = Vector((-0.02, s * 0.118, 0.745))
        RB = Vector((-0.19, s * 0.159, 0.52))
        d = (RB - B).normalized()
        o = Vector((0, s * 1.0, -0.35))
        o = (o - d * o.dot(d)).normalized() * (TUBE_R + 0.0052)
        q = [B + d * 0.035 + o, B + d * 0.12 + o, B + d * 0.20 + o, B + d * 0.228 + o]
        loom(g, q, 0.0040, ties=[(0.2, 0), (0.55, 0), (0.9, 0)], fillet=0.0)
        box(g, "plastic_black", 0.012, 0.009, 0.014, B + d * 0.238 + o, d, (0, s, 0), bevel=0.0012)
    # clutch cable along the left front trellis tube with P-clips and a barrel adjuster
    A = Vector((0.250, 0.118, 0.745))
    HB = Vector((0.10, 0.180, 0.615))
    d = (HB - A).normalized()
    o = Vector((0, 1, -0.3))
    o = (o - d * o.dot(d)).normalized()
    oc = o * (TUBE_R + 0.0042)
    cp = [A + d * 0.035 + oc, A + d * 0.10 + oc, A + d * 0.165 + oc, A + d * 0.185 + oc + Vector((0, 0.004, -0.012)),
          Vector((0.128, 0.193, 0.585))]
    tube(g, "plastic_black", cp, 0.0029, 12, fillet=0.012)
    for u in (0.065, 0.135):
        c = A + d * u + oc
        torus(g, "steel_polished", 0.0039, 0.0009, c, d, 16, 4)
        tube(g, "steel_polished", [c - o * 0.0040, A + d * u + o * (TUBE_R - 0.0005)], 0.0012, 8)
    cyl(g, "ano_blue", 0.0042, 0.012, (0.128, 0.193, 0.579), (0, 0, -1), 16, bevel=0.0006)
    cyl(g, "steel_polished", 0.0050, 0.003, (0.128, 0.193, 0.5715), (0, 0, -1), 6)
    cyl(g, "steel_polished", 0.0019, 0.010, (0.128, 0.193, 0.565), (0, 0, -1), 10)
    # ignition coils side by side under the fuel cell, front
    for s in (1, -1):
        cx, cy, cz = 0.150, s * 0.052, 0.690
        box(g, "plastic_black", 0.056, 0.030, 0.024, (cx, cy, cz), bevel=0.003)
        for k in range(5):
            box(g, "plastic_black", 0.0025, 0.032, 0.026, (cx - 0.020 + 0.010 * k, cy, cz), bevel=0.0008)
        cyl(g, "rubber", 0.0068, 0.016, (cx + 0.032, cy, cz - 0.002), (1, 0, 0), 20, bevel=0.0015)
        tube(g, "rubber", [(cx + 0.040, cy, cz - 0.002), (cx + 0.052, cy, cz - 0.006), (cx + 0.060, cy * 1.1, cz - 0.016)],
             0.0036, 12, fillet=0.006)
        box(g, "alu_machined", 0.064, 0.020, 0.0025, (cx, cy, cz + 0.0135), bevel=0.0006)
        for dx in (-0.026, 0.026):
            bolt(g, (cx + dx, cy, cz + 0.0148), (0, 0, 1), 0.0035, "hex", "zinc")
    # finned regulator / rectifier under the fuel cell, rear
    rx, rz = -0.150, 0.693
    box(g, "engine_fin", 0.070, 0.060, 0.010, (rx, 0.0, rz + 0.004), bevel=0.0015)
    for k in range(9):
        box(g, "engine_fin", 0.066, 0.0022, 0.012, (rx, -0.028 + 0.007 * k, rz - 0.007), bevel=0.0006)
    box(g, "plastic_black", 0.016, 0.022, 0.009, (rx + 0.040, 0.0, rz + 0.003), bevel=0.0012)
    tube(g, "plastic_black", [(rx + 0.048, 0.0, rz + 0.003), (-0.090, 0.050, 0.699), (-0.078, 0.0935, 0.708),
                              (-0.075, 0.0955, 0.722)], 0.0030, 12, fillet=0.008)
    for (dx, dy) in ((-0.030, -0.025), (-0.030, 0.025), (0.030, -0.025), (0.030, 0.025)):
        cyl(g, "steel_dark", 0.0035, 0.004, (rx + dx, dy, rz + 0.011), (0, 0, 1), 6)
    # relay / fuse box hung under the rear cross tube
    fx, fz = -0.283, 0.713
    box(g, "plastic_black", 0.040, 0.070, 0.026, (fx, 0.0, fz), bevel=0.003)
    box(g, "plastic_black", 0.036, 0.066, 0.004, (fx, 0.0, fz + 0.0145), bevel=0.0012)
    for k in range(4):
        box(g, "ano_red", 0.006, 0.010, 0.004, (fx - 0.010 + 0.0, -0.024 + 0.016 * k, fz + 0.0175), bevel=0.0006)
    box(g, "ano_blue", 0.006, 0.010, 0.004, (fx + 0.010, -0.016, fz + 0.0175), bevel=0.0006)
    box(g, "ano_gold", 0.006, 0.010, 0.004, (fx + 0.010, 0.008, fz + 0.0175), bevel=0.0006)
    box(g, "alu_machined", 0.017, 0.003, 0.040, (-0.3155, 0.030, 0.727), bevel=0.0006)
    box(g, "alu_machined", 0.017, 0.003, 0.040, (-0.3155, -0.030, 0.727), bevel=0.0006)
    # braided fuel feed from the cell front face down to the engine, AN fittings
    fy = -0.015
    cyl(g, "ano_red", 0.0058, 0.006, (0.193, fy, 0.716), (1, 0, 0), 6, bevel=0.0005)
    cyl(g, "ano_blue", 0.0050, 0.008, (0.200, fy, 0.716), (1, 0, 0), 6, bevel=0.0005)
    tube(g, "braid", [(0.203, fy, 0.716), (0.209, fy, 0.708), (0.208, fy, 0.692), (0.200, fy, 0.684),
                      (0.197, fy, 0.679)], 0.0037, 12, fillet=0.007)
    cyl(g, "ano_blue", 0.0050, 0.008, (0.197, fy, 0.677), (0, 0, -1), 6, bevel=0.0005)
    cyl(g, "ano_red", 0.0056, 0.005, (0.197, fy, 0.6705), (0, 0, -1), 6, bevel=0.0005)
    # fuel cell: level sight windows with bezels + rivet rows on both side faces
    XZ = ((1, 0, 0), (0, 0, 1))
    for s in (1, -1):
        cxw, czw = -0.030, 0.742
        ring(g, "alu_machined", rrect(cxw, czw, 0.106, 0.019, 0.005), rrect(cxw, czw, 0.094, 0.010, 0.003),
             (cxw, czw), 0.0012, (0, s * 0.0880, 0), XZ[0], XZ[1], n=64, ch=0.0004, out=(0, s, 0))
        box(g, "amber_fluid", 0.093, 0.0006, 0.0055, (cxw, s * 0.0883, czw - 0.0022))
        box(g, "glass", 0.0935, 0.0005, 0.0098, (cxw, s * 0.0889, czw))
        for (dx, dz) in ((-0.0495, 0.0), (0.0495, 0.0), (-0.025, 0.0073), (0.025, 0.0073), (-0.025, -0.0073),
                         (0.025, -0.0073)):
            lathe(g, "steel_polished", [(0.0, 0.0009), (0.0010, 0.0007), (0.0013, 0.0), (0.0, 0.0)],
                  (cxw + dx, s * 0.0892, czw + dz), (0, s, 0), 10)
        for k in range(17):
            x = -0.235 + 0.025 * k
            if abs(x + 0.19) < 0.016 or abs(x - 0.13) < 0.016:
                continue
            for z in (0.7135, 0.7505):
                if z > 0.745 and abs(x - cxw) < 0.06:
                    continue
                lathe(g, "steel_polished", [(0.0, 0.0010), (0.0011, 0.0008), (0.0015, 0.0), (0.0, 0.0)],
                      (x, s * 0.0880, z), (0, s, 0), 10)
    # strap bolts on the fuel cell straps
    for sx in (-0.19, 0.13):
        for s in (1, -1):
            bolt(g, (sx, s * 0.091, 0.748), (0, s, 0), 0.0040, "hex", "steel_polished", washer=True)
    # horn under the tower cross tube
    hx, hz = 0.229, 0.678
    lathe(g, "ano_black", [(0.0, -0.011), (0.022, -0.011), (0.024, -0.006), (0.024, 0.004), (0.020, 0.008),
                           (0.010, 0.011), (0.0, 0.011)], (hx, 0.0, hz), (1, 0, 0), 40)
    for k in range(5):
        torus(g, "ano_black", 0.006 + 0.0032 * k, 0.0006, (hx + 0.0105 - 0.0005 * k, 0.0, hz), (1, 0, 0), 32, 4)
    cyl(g, "steel_polished", 0.004, 0.003, (hx + 0.0115, 0.0, hz), (1, 0, 0), 16)
    box(g, "steel_brushed", 0.003, 0.010, 0.026, (0.2195, 0.0, 0.6985), bevel=0.0005)
    torus(g, "steel_brushed", 0.0115, 0.0014, (0.232, 0.0, 0.715), (0, 1, 0), 24, 6)
    # catch can on the left under the rail
    kx, ky, kz = -0.130, 0.119, 0.700
    lathe(g, "alu_machined", [(0.0, -0.022), (0.013, -0.022), (0.015, -0.019), (0.015, 0.018), (0.013, 0.021),
                              (0.0, 0.021)], (kx, ky, kz), (0, 0, 1), 40)
    for k in range(2):
        torus(g, "alu_machined", 0.0151, 0.0006, (kx, ky, kz - 0.014 + 0.004 * k), (0, 0, 1), 40, 4)
    cyl(g, "ano_red", 0.0130, 0.004, (kx, ky, kz + 0.023), (0, 0, 1), 36, bevel=0.0008)
    cyl(g, "brass", 0.0032, 0.010, (kx + 0.008, ky - 0.006, kz + 0.028), (0, 0, 1), 16)
    torus(g, "steel_brushed", 0.0158, 0.0018, (kx, ky, kz + 0.009), (0, 0, 1), 32, 6)
    box(g, "steel_brushed", 0.010, 0.003, 0.038, (kx, ky + 0.0166, kz + 0.026), bevel=0.0005)
    cyl(g, "brass", 0.0030, 0.008, (kx, ky - 0.017, kz - 0.010), (0, -1, 0), 16)
    tube(g, "rubber", [(kx, ky - 0.019, kz - 0.010), (kx + 0.003, ky - 0.025, kz - 0.015), (-0.115, 0.085, 0.676),
                       (-0.110, 0.082, 0.668)], 0.0032, 12, fillet=0.006)
    torus(g, "zinc", 0.0040, 0.0010, (kx, ky - 0.0195, kz - 0.010), (0, -1, 0), 16, 4)
    torus(g, "zinc", 0.0040, 0.0010, (-0.112, 0.0835, 0.672), (0.4, -0.2, -0.9), 16, 4)


# ----------------------------------------------------------------------------- carriage
def build_carriage():
    g = "CARR"
    x0, x1 = -0.745, -0.32
    # seat edge trim (aluminium bead following the seat base) and stitching welt at the rear
    for s in (1, -1):
        pts = []
        for i in range(0, 41):
            x = x0 + 0.004 + (x1 - x0 - 0.016) * i / 40
            u = (x - x0) / (x1 - x0)
            hw = (0.104 + 0.022 * u) * 0.93 + 0.0012
            pts.append((x, s * hw, 0.7955))
        tube(g, "alu_brushed", pts, 0.0021, 10)
    rear = []
    for k in range(25):
        y = -0.0952 + 0.1904 * k / 24
        rear.append((x0 - 0.0010, y, 0.7955))
    tube(g, "alu_brushed", rear, 0.0021, 10)
    # seat pan rivets underneath
    for s in (1, -1):
        for k in range(11):
            x = -0.722 + 0.0375 * k
            lathe(g, "steel_polished", [(0.0, -0.0013), (0.0013, -0.0010), (0.0019, 0.0), (0.0, 0.0)],
                  (x, s * 0.086, 0.787), (0, 0, 1), 12)
    # quarter-turn fasteners on the pan corners
    for (x, y) in ((-0.715, 0.06), (-0.715, -0.06), (-0.36, 0.06), (-0.36, -0.06)):
        lathe(g, "steel_polished", [(0.0, -0.0022), (0.0040, -0.0018), (0.0052, -0.0006), (0.0052, 0.0), (0.0, 0.0)],
              (x, y, 0.787), (0, 0, 1), 24)
        box(g, "steel_dark", 0.0075, 0.0011, 0.0010, (x, y, 0.7848))
    # perforated splash guard under the rear rails (between the rails, clear of the guide blocks)
    gx0, gx1, gy = -1.034, -0.828, 0.074
    zb = 0.7628
    t = 0.0022
    pitch = 0.0095
    nx = int((gx1 - gx0 - 0.016) / pitch)
    ny = int((2 * gy - 0.016) / pitch)
    ox = (gx0 + gx1) / 2 - (nx - 1) * pitch / 2
    oy = -(ny - 1) * pitch / 2
    perfs = []
    for i in range(nx):
        for j in range(ny):
            cx, cy = ox + i * pitch, oy + j * pitch
            sq = [(cx - pitch / 2, cy - pitch / 2), (cx + pitch / 2, cy - pitch / 2), (cx + pitch / 2, cy + pitch / 2),
                  (cx - pitch / 2, cy + pitch / 2)]
            perfs.append(ring(g, "alu_plate", sq, circle(cx, cy, 0.0029, 12), (cx, cy), t, (0, 0, zb), (1, 0, 0),
                              (0, 1, 0), n=12, ch=0.0002))
    fx0 = ox - pitch / 2
    fx1 = ox + (nx - 1) * pitch + pitch / 2
    fy = oy - pitch / 2
    outer = rrect((gx0 + gx1) / 2, 0.0, gx1 - gx0, 2 * gy, 0.008)
    inner = [(fx0, fy), (fx1, fy), (fx1, -fy), (fx0, -fy)]
    ring(g, "alu_plate", outer, inner, ((gx0 + gx1) / 2, 0.0), t, (0, 0, zb), (1, 0, 0), (0, 1, 0), n=64, ch=0.0003)
    for s in (1, -1):
        box(g, "alu_plate", gx1 - gx0 - 0.01, 0.0022, 0.009, ((gx0 + gx1) / 2, s * (gy - 0.0011), zb + 0.0045),
            bevel=0.0006)
        # hanger clamps on the cross tubes
        for xc in (-1.045, -0.86):
            torus(g, "steel_brushed", 0.0092, 0.0013, (xc, s * 0.050, 0.775), (0, 1, 0), 24, 6)
            box(g, "steel_brushed", 0.008, 0.007, 0.0105, (xc + (0.010 if xc < -1 else 0.0), s * 0.050, 0.7685),
                bevel=0.0006)
        for k in range(4):
            lathe(g, "steel_polished", [(0.0, 0.0), (0.0030, 0.0), (0.0024, 0.0012), (0.0, 0.0016)],
                  (gx0 + 0.012 + k * (gx1 - gx0 - 0.024) / 3, s * (gy - 0.006), zb + t), (0, 0, 1), 12)
    # tail light housing: cooling fins on top, screws, end plugs
    box(g, "ano_black", 0.024, 0.190, 0.0016, (-1.062, 0.0, 0.8297), bevel=0.0004)
    for k in range(17):
        y = -0.088 + 0.011 * k
        box(g, "ano_black", 0.020, 0.0015, 0.0042, (-1.062, y, 0.8318), bevel=0.0004)
    for y in (-0.105, 0.105):
        bolt(g, (-1.062, y, 0.829), (0, 0, 1), 0.0035, "button", "steel_polished")
    for y in (-0.1115, 0.1115):
        bolt(g, (-1.077, y, 0.8205), (-1, 0, 0), 0.0028, "socket", "steel_polished")
        bolt(g, (-1.077, y, 0.8035), (-1, 0, 0), 0.0028, "socket", "steel_polished")
    # LED turn signal pods with amber lenses
    for s in (1, -1):
        py, pz = s * 0.137, 0.812
        lathe(g, "ano_black", [(0.0, -0.011), (0.0070, -0.011), (0.0085, -0.006), (0.0085, 0.008), (0.0080, 0.010),
                               (0.0, 0.010)], (-1.064, py, pz), (1, 0, 0), 28)
        lathe(g, "amber_fluid", [(0.0, 0.0), (0.0075, 0.0), (0.0072, 0.0020), (0.0055, 0.0034), (0.0, 0.0042)],
              (-1.0745, py, pz), (-1, 0, 0), 28)
        torus(g, "alu_machined", 0.0081, 0.0007, (-1.0745, py, pz), (1, 0, 0), 28, 4)
        cyl(g, "ano_black", 0.0035, 0.024, (-1.062, s * 0.122, pz), (0, 1, 0), 16)
    # reflector under the tail light
    box(g, "ano_black", 0.006, 0.056, 0.016, (-1.071, 0.0, 0.7865), bevel=0.0012)
    box(g, "glass_red", 0.0025, 0.050, 0.011, (-1.0745, 0.0, 0.7865), bevel=0.0008)
    box(g, "ano_black", 0.010, 0.010, 0.012, (-1.064, 0.0, 0.7965), bevel=0.0008)
    # tail light lead: along the inside of the left rail, clipped to the cross tubes, into a connector
    wy = 0.068
    wp = [(-1.050, 0.060, 0.805), (-1.046, wy, 0.792), (-1.020, wy, 0.789), (-0.86, wy, 0.789), (-0.775, wy, 0.789)]
    tube(g, "plastic_black", wp, 0.0024, 10, fillet=0.006)
    for xc in (-1.040, -0.86):
        torus(g, "steel_brushed", 0.0034, 0.0008, (xc, wy, 0.789), (1, 0, 0), 16, 4)
        box(g, "steel_brushed", 0.006, 0.0016, 0.010, (xc, wy - 0.0040, 0.7835))
    box(g, "plastic_black", 0.020, 0.012, 0.010, (-0.765, 0.066, 0.7885), bevel=0.0012)
    # carriage rail end plugs
    for s in (1, -1):
        for xe in (-1.06, -0.30):
            n = (-1, 0, 0) if xe < -0.5 else (1, 0, 0)
            cyl(g, "alu_machined", 0.0105, 0.003, (xe + n[0] * 0.0015, s * 0.10, 0.775), n, 32, bevel=0.0007)
            bolt(g, (xe + n[0] * 0.003, s * 0.10, 0.775), n, 0.0045, "socket", "steel_dark")


# ----------------------------------------------------------------------------- swingarm pivot block / crank
def build_sap():
    g = "SAP"
    px, pz = C.PIVOT[0], C.PIVOT[2]
    for (y0, y1) in ((0.0660, 0.0866), (0.1734, 0.1856)):
        L = y1 - y0
        nr = 5 if L > 0.015 else 3
        prof = [(0.0134, 0.0)]
        for k in range(2 * nr + 1):
            z = L * k / (2 * nr)
            r = 0.0195 if k % 2 == 1 else 0.0160
            if k == 0 or k == 2 * nr:
                r = 0.0172
            prof.append((r, z))
        prof.append((0.0134, L))
        lathe(g, "rubber", prof, (px, y0, pz), (0, 1, 0), 32, smooth=50.0)
        for yy in (y0 + 0.0012, y1 - 0.0012):
            torus(g, "steel_polished", 0.0174, 0.0009, (px, yy, pz), (0, 1, 0), 32, 6)
    # crank: bearing cap + bolts on the outer face, pocket rim
    cyl(g, "alu_machined", 0.0165, 0.0030, (px, 0.2325, pz), (0, 1, 0), 40, bevel=0.0008)
    bolt(g, (px, 0.234, pz), (0, 1, 0), 0.0060, "socket", "steel_polished")
    for ang in (0, 120, 240):
        a = math.radians(ang + 90)
        bolt(g, (px + 0.0118 * math.cos(a), 0.234, pz + 0.0118 * math.sin(a)), (0, 1, 0), 0.0024, "socket", "steel_dark")
    # grease nipple on the block (rear face, below the link rails)
    cyl(g, "brass", 0.0028, 0.006, (px - 0.035 - 0.003, 0.165, pz - 0.018), (-1, 0, 0), 6)
    lathe(g, "brass", [(0.0, 0.0), (0.0018, 0.0), (0.0026, 0.002), (0.0018, 0.004), (0.0, 0.0045)],
          (px - 0.041, 0.165, pz - 0.018), (-1, 0, 0), 12)


# ----------------------------------------------------------------------------- links (local frames)
def build_l1():
    g = "L1"
    L = C.LINK1
    # pockets + bolt rows on the outer faces of both rails (z = +-0.051), within 2 mm
    for sz in (1, -1):
        zf = sz * 0.051
        U, V = (1, 0, 0), (0, sz * 1.0, 0)
        for cx in (-0.105, -0.215):
            pocket(g, (0, 0, zf), (1, 0, 0), (0, 1, 0), (cx, 0.0), 0.080, 0.030, 0.008, rim=0.0024, t=0.0010,
                   floor_mat="ano_black", out=(0, 0, sz))
        for x in (-0.042, -0.160, -0.270):
            for yy in (-0.0135, 0.0135):
                bolt(g, (x, yy, zf), (0, 0, sz), 0.0036, "socket", "steel_dark")
        # grease nipples near both pins
        for x in (-0.030, -L + 0.034):
            cyl(g, "zinc", 0.0030, 0.003, (x, 0.0, zf + sz * 0.0015), (0, 0, sz), 6)
            lathe(g, "brass", [(0.0, 0.0), (0.0016, 0.0), (0.0024, 0.0022), (0.0016, 0.0042), (0.0, 0.0048)],
                  (x, 0.0, zf + sz * 0.003), (0, 0, sz), 12)
        # corrugated dust boots on the pin ends
        for x in (0.0, -L):
            prof = [(0.0, 0.0), (0.0128, 0.0)]
            for k in range(7):
                prof.append((0.0132 if k % 2 == 1 else 0.0117, 0.0008 + 0.0040 * k / 6))
            prof.append((0.0080, 0.0055))
            prof.append((0.0, 0.0055))
            lathe(g, "rubber", prof, (x, 0.0, sz * 0.057), (0, 0, sz), 28, closed=False, smooth=50.0)
    # web: pocket rims between the existing bolts
    for cx in (-0.109, -0.157, -0.205):
        pocket(g, (0, -0.025, 0), (1, 0, 0), (0, 0, 1), (cx, 0.0), 0.030, 0.016, 0.005,
               rim=0.0020, t=0.0008, out=(0, -1, 0))
    # rigid brake line on the web side with P clips; ends with fittings clear of the block / elbow sweeps
    yl, zl = -0.0300, -0.0125
    tube(g, "steel_polished", [(-0.060, yl, zl), (-0.268, yl, zl)], 0.0021, 10)
    for x in (-0.080, -0.170, -0.245):
        torus(g, "steel_brushed", 0.0031, 0.0008, (x, yl, zl), (1, 0, 0), 16, 4)
        box(g, "steel_brushed", 0.006, 0.0045, 0.0016, (x, -0.0275, zl - 0.0039))
    for (p, n) in (((-0.059, yl, zl), (1, 0, 0)), ((-0.272, yl, zl), (-1, 0, 0))):
        cyl(g, "brass", 0.0040, 0.008, p, n, 6, bevel=0.0004)


def build_l2():
    g = "L2"
    yf = -0.022
    # brake line segment on the -Y face (outer when folded), from the elbow to the caliper flange
    bl = [(-0.022, yf - 0.0045, 0.017), (-0.070, yf - 0.0045, 0.017), (-0.090, yf - 0.0045, 0.010)]
    tube(g, "steel_polished", bl, 0.0021, 10, fillet=0.010)
    cyl(g, "brass", 0.0040, 0.008, (-0.026, yf - 0.0045, 0.017), (1, 0, 0), 6, bevel=0.0004)
    cyl(g, "steel_polished", 0.0045, 0.007, (-0.094, yf - 0.0045, 0.0085), (-1, 0, -0.4), 6, bevel=0.0004)
    for x in (-0.045, -0.072):
        torus(g, "steel_brushed", 0.0031, 0.0008, (x, yf - 0.0045, 0.017), (1, 0, 0), 16, 4)
        bolt(g, (x, yf, 0.0233), (0, -1, 0), 0.0028, "socket", "steel_dark")
    # torque-arm style brace with bolted ends
    prism(g, "ano_orange", stadium((-0.030, -0.013), (-0.090, -0.016), 0.0068, 0.0068), 0.004,
          (0, yf - 0.002, 0), (1, 0, 0), (0, 0, 1), bevel=0.0007)
    prism(g, "alu_machined", stadium((-0.042, -0.0135), (-0.078, -0.0155), 0.0028, 0.0028), 0.0006,
          (0, yf - 0.0043, 0), (1, 0, 0), (0, 0, 1), bevel=0.0002)
    for (x, z) in ((-0.030, -0.013), (-0.090, -0.016)):
        bolt(g, (x, yf - 0.004, z), (0, -1, 0), 0.0045, "hex", "steel_polished", washer=True)
    # pinch bolts by the flange
    for z in (0.024, -0.025):
        bolt(g, (-0.099, yf, z), (0, -1, 0), 0.0032, "socket", "steel_dark")


# ----------------------------------------------------------------------------- head carrier (fork frame)
def build_head():
    g = "HEAD"
    for s in (1, -1):
        yf = s * 0.059
        pocket(g, (0, yf, 0), (1, 0, 0), (0, 0, 1), (-0.044, -0.0565), 0.030, 0.0135, 0.0045, rim=0.0018, t=0.0008,
               out=(0, s, 0))
        for (x, z) in ((-0.027, -0.044), (-0.086, -0.0535)):
            bolt(g, (x, yf, z), (0, s, 0), 0.0040, "socket", "steel_polished", washer=True)
        # steering stop blocks under the lower lug
        box(g, "alu_machined", 0.012, 0.014, 0.006, (-0.036, s * 0.044, -0.1695), bevel=0.0012)
        box(g, "rubber", 0.004, 0.012, 0.0045, (-0.0285, s * 0.044, -0.1697), bevel=0.0008)
        bolt(g, (-0.036, s * 0.051, -0.1695), (0, s, 0), 0.0025, "socket", "steel_dark")
    # cable guide loop on the back of the lower lug
    torus(g, "steel_polished", 0.0055, 0.0012, (-0.0845, 0.0, -0.150), (0, 0, 1), 20, 6)
    box(g, "steel_polished", 0.010, 0.008, 0.0024, (-0.077, 0.0, -0.1505), bevel=0.0005)


# ----------------------------------------------------------------------------- keep-out checker
SH_A = Vector((-0.05, 0.215, 0.672))
SH_B = Vector((-0.275, 0.215, 0.52))
ANCH = [Vector(v) for v in ((-0.30, 0.196, 0.615), (-0.30, -0.196, 0.615), (-0.262, 0.226, 0.555),
                            (-0.262, -0.226, 0.555))]


def _seg_d(p, a, b):
    ab = b - a
    t = max(0.0, min(1.0, (p - a).dot(ab) / ab.length_squared))
    return (p - (a + ab * t)).length


def _zones_ch(p):
    x, y, z = p
    ay = abs(y)
    out = []
    if x < -0.325 and ay < 0.35 and 0.07 < z < 0.74:
        out.append("K1")
    if x > 0.262 and z < 0.75:
        out.append("K2")
    if -0.335 < x < -0.25 and -0.23 < y < 0.17 and 0.33 < z < 0.49:
        out.append("K3")
    if ay < 0.135 and 0.762 < z < 0.93 and -0.56 < x < 0.26:
        out.append("K4")
    if z > 0.695 and not (ay < 0.18 and z < 0.915 and -0.36 <= x <= 0.30):
        out.append("K5")
    if -0.70 <= x <= -0.33 and ay < 0.145 and 0.685 <= z <= 0.762:
        out.append("K6")
    if ay < 0.19 and z < 0.172 and -0.30 <= x <= 0.36:
        out.append("K7")
    if -0.50 <= x <= -0.15 and 0.32 <= z <= 0.72:
        if z >= 0.42 and ay > 0.225:
            out.append("K8")
        if z < 0.42 and ay > 0.31:
            out.append("K8")
    if _seg_d(p, SH_A, SH_B) < 0.02 or any((p - a).length < 0.02 for a in ANCH):
        out.append("K9")
    if ay > 0.31:
        out.append("K10")
    if -0.02 <= x <= 0.56 and math.hypot(ay - 0.535, z) < 0.38:
        out.append("K11")
    return out


def _zones_carr(p):
    x, y, z = p
    out = []
    if -0.70 <= x <= -0.33 and abs(y) < 0.145 and 0.685 <= z <= 0.762:
        out.append("K6")
    if abs(y) < 0.19 and z < 0.172 and -0.30 <= x <= 0.36:
        out.append("K7")
    if abs(y) > 0.16 or z > 0.93 or x < -1.085 or (x > -0.80 and z < 0.762):
        out.append("K10")
    return out


def check(res):
    """Vertex test of the CH / CARR / SAP lists against the keep-out zones. Returns the number of violating verts."""
    dg = bpy.context.evaluated_depsgraph_get()
    bad = 0
    for key, fn in (("CH", _zones_ch), ("CARR", _zones_carr), ("SAP", lambda p: ["K7"] if p.z < 0.172 else [])):
        for o in res.get(key, []):
            ev = o.evaluated_get(dg)
            me = ev.to_mesh()
            hits = {}
            for v in me.vertices:
                p = o.matrix_world @ v.co
                for k in fn(p):
                    if k not in hits:
                        hits[k] = [0, tuple(round(c, 4) for c in p)]
                    hits[k][0] += 1
            ev.to_mesh_clear()
            for k, (n, ex) in hits.items():
                bad += n
                print("CD-CHECK %s %s: %d verts in %s, e.g. %s" % (key, o.name, n, k, ex))
    print("CD-CHECK done, %d violating vertices" % bad)
    return bad


# ----------------------------------------------------------------------------- build
def build(col):
    global COL, G
    COL = col
    G = {}
    steps = (("frame", build_frame), ("carriage", build_carriage), ("sap", build_sap), ("l1", build_l1),
             ("l2", build_l2), ("head", build_head))
    for nm, fn in steps:
        try:
            fn()
        except Exception as e:                    # keep whatever else works
            import traceback
            traceback.print_exc()
            print("CD step %s failed: %s" % (nm, e))
    # join per group into a few multi-material meshes (metal / non-metal) to keep the object count low
    soft = {"plastic_black", "rubber", "carbon", "braid", "glass", "amber_fluid", "glass_red", "copper", "engine_fin"}
    res = {}
    for grp, bym in G.items():
        lst = []
        for fam in ("metal", "soft"):
            objs = []
            for mat, ol in bym.items():
                if (mat in soft) == (fam == "soft"):
                    objs += [o for o in ol if o is not None]
            if objs:
                lst.append(geo.join(objs, "cd_%s_%s" % (grp, fam)))
        res[grp] = lst
    try:
        allo = [o for v in res.values() for o in v]
        print("CD objects %d  tris %d" % (len(allo), geo.tri_count(allo)))
        check(res)
    except Exception as e:
        print("CD check failed", e)
    return res
