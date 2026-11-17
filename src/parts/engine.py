"""Engine + exhaust of the cafe racer: air-cooled parallel twin, unit construction, 2-into-1 exhaust.

build(col) -> {"engine": root}; root is an Empty at the world origin, everything is modelled in the
final-pose world frame (X forward, Y left, Z up) and parented to it.

Layout (world): crank axis (0.04, y, 0.36), cylinders lean 12 deg forward, gearbox behind the crank,
bevel-drive output flange at (-0.250, +0.13, 0.405) facing -X, exhaust can along X at y -0.255, z 0.205.
Cylinder parts are built in a local frame (u forward, y left, d along the cylinder axis) and mapped by M_CYL.
"""
import math
import bpy
import bmesh
from mathutils import Vector, Matrix

import geo
import mats

TILT = math.radians(12.0)
CRANK = Vector((0.04, 0.0, 0.36))
M_CYL = Matrix.Translation(CRANK) @ Matrix.Rotation(TILT, 4, "Y")
R_CYL = M_CYL.to_3x3()
RX90 = Matrix.Rotation(math.pi / 2, 4, "X")                                   # (a, b, z) -> (a, -z, b)
M_YZ = Matrix(((0, 0, 1, 0), (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 0, 1)))       # (a, b, z) -> (z, a, b)

BK = "engine_black"
SP = "steel_polished"
SD = "steel_dark"
AM = "alu_machined"

# frame mount hard points (x, casting-surface |y|, z); boss face is 0.008 further out
MOUNTS = {"front_lower": (0.13, 0.135, 0.33), "rear_upper": (-0.19, 0.135, 0.52), "head": (0.10, 0.156, 0.615)}


def Lp(u, y, d):
    """Cylinder-local point -> world."""
    return M_CYL @ Vector((u, y, d))


def Ld(u, y, d):
    return (R_CYL @ Vector((u, y, d))).normalized()


# ----------------------------------------------------------------------------- small geometry helpers
def xf(ob, m):
    ob.data.transform(m)
    ob.data.update()
    return ob


def orient_m(pos, dirv):
    """Matrix taking +Z to dirv and the origin to pos."""
    d = Vector(dirv).normalized()
    if d.z < -0.99999:
        rot = Matrix.Rotation(math.pi, 4, "X")
    else:
        rot = Vector((0, 0, 1)).rotation_difference(d).to_matrix().to_4x4()
    return Matrix.Translation(Vector(pos)) @ rot


def rprof(corners, res=3):
    """Lathe profile from corner points (r, z[, t]); t = tangent length of the corner rounding."""
    pts = []
    n = len(corners)
    for i, c in enumerate(corners):
        p = Vector((c[0], c[1]))
        t = c[2] if len(c) > 2 else 0.0
        if t <= 0 or i == 0 or i == n - 1:
            pts.append((p.x, p.y))
            continue
        d1 = Vector(corners[i - 1][:2]) - p
        d2 = Vector(corners[i + 1][:2]) - p
        t = min(t, d1.length * 0.45, d2.length * 0.45)
        p1 = p + d1.normalized() * t
        p2 = p + d2.normalized() * t
        for j in range(res + 1):
            u = j / res
            q = (1 - u) ** 2 * p1 + 2 * u * (1 - u) * p + u ** 2 * p2
            pts.append((q.x, q.y))
    return pts


def ccw(pts):
    a = sum(pts[i - 1][0] * pts[i][1] - pts[i][0] * pts[i - 1][1] for i in range(len(pts)))
    return list(pts) if a > 0 else list(reversed(pts))


def offset_poly(pts, b):
    """Inward offset of a CCW polygon by b (mitred)."""
    n = len(pts)
    out = []
    for i in range(n):
        p0, p, p2 = Vector(pts[i - 1]), Vector(pts[i]), Vector(pts[(i + 1) % n])
        e1, e2 = p - p0, p2 - p
        if e1.length < 1e-10:
            e1 = e2.copy()
        if e2.length < 1e-10:
            e2 = e1.copy()
        e1.normalize()
        e2.normalize()
        n1 = Vector((e1.y, -e1.x))
        n2 = Vector((e2.y, -e2.x))
        m = n1 + n2
        if m.length < 1e-9:
            m = n1.copy()
        m.normalize()
        out.append(p - m * (b / max(m.dot(n1), 0.5)))
    return out


def round_poly(pts, rad, res=4):
    pts = ccw(pts)
    n = len(pts)
    out = []
    for i in range(n):
        p, a, b = Vector(pts[i]), Vector(pts[i - 1]), Vector(pts[(i + 1) % n])
        d1, d2 = a - p, b - p
        t = min(rad, d1.length * 0.45, d2.length * 0.45)
        p1 = p + d1.normalized() * t
        p2 = p + d2.normalized() * t
        for j in range(res + 1):
            u = j / res
            q = (1 - u) ** 2 * p1 + 2 * u * (1 - u) * p + u ** 2 * p2
            out.append((q.x, q.y))
    return out


def rrect(hx, hy, r, n=6, step=0.0):
    """CCW rounded rectangle centred on the origin; step > 0 subdivides the straight edges."""
    pts = []
    for cx, cy, a0 in ((hx - r, hy - r, 0.0), (-hx + r, hy - r, 90.0), (-hx + r, -hy + r, 180.0), (hx - r, -hy + r, 270.0)):
        for j in range(n + 1):
            a = math.radians(a0 + 90.0 * j / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    if step > 0:
        out = []
        m = len(pts)
        for i in range(m):
            p, q = Vector(pts[i]), Vector(pts[(i + 1) % m])
            out.append((p.x, p.y))
            cnt = int((q - p).length / step)
            for j in range(1, cnt):
                w = p.lerp(q, j / cnt)
                out.append((w.x, w.y))
        pts = out
    return pts


def fin_outline(hd_f, hd_r, hw, r, notch_w=0.022, notch_d=0.013):
    """Cooling fin outline in (u, y): rounded rectangle with a scallop between the cylinders front and rear."""
    hx = (hd_f + hd_r) / 2
    cx = (hd_f - hd_r) / 2
    out = []
    for a, b in rrect(hx, hw, r, 8, 0.005):
        a += cx
        if abs(b) < notch_w:
            dd = notch_d * 0.5 * (1 + math.cos(math.pi * b / notch_w))
            if a > hd_f - 1e-6:
                a -= dd
            elif a < -hd_r + 1e-6:
                a += dd
        out.append((a, b))
    return out


def hull_outline(circles, n=72):
    """Convex hull (CCW) of circles (cx, cy, r)."""
    pts = []
    for cx, cy, r in circles:
        for i in range(n):
            a = 2 * math.pi * i / n
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    pts = sorted(set(pts))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def along_outline(pts, count, inset, phase=0.5):
    """count points evenly spaced along a CCW outline, moved inward by inset."""
    pts = ccw(pts)
    m = len(pts)
    seg = [(Vector(pts[(i + 1) % m]) - Vector(pts[i])).length for i in range(m)]
    total = sum(seg)
    out = []
    for j in range(count):
        s = (j + phase) / count * total
        i = 0
        while s > seg[i]:
            s -= seg[i]
            i += 1
        a, b = Vector(pts[i]), Vector(pts[(i + 1) % m])
        t = (b - a).normalized()
        p = a + t * s - Vector((t.y, -t.x)) * inset
        out.append((p.x, p.y))
    return out


def plate_bm(bm, outline, z0, z1, b=0.0, n=2):
    """Add an extruded plate (outline in XY, z0..z1) with rounded (n segments) rim edges to bm."""
    pts = ccw(outline)
    rings = []
    if b > 0:
        for j in range(n + 1):
            a = (math.pi / 2) * j / n
            rings.append((offset_poly(pts, b * (1 - math.sin(a))), z0 + b * (1 - math.cos(a))))
        for j in range(n, -1, -1):
            a = (math.pi / 2) * j / n
            rings.append((offset_poly(pts, b * (1 - math.sin(a))), z1 - b * (1 - math.cos(a))))
    else:
        rings = [(pts, z0), (pts, z1)]
    vr = [[bm.verts.new((p[0], p[1], z)) for p in ring] for ring, z in rings]
    m = len(pts)
    for ra, rb in zip(vr[:-1], vr[1:]):
        for i in range(m):
            j = (i + 1) % m
            bm.faces.new((ra[i], ra[j], rb[j], rb[i]))
    bm.faces.new(list(reversed(vr[0])))
    bm.faces.new(vr[-1])


def lathe_x(name, prof, cy, cz, col, segs=64, smooth=40.0):
    """Revolve (x, r[, k]) around an X-parallel axis through (cy, cz); k shears the ring along X with
    height (slash cut): x += k * (z - cz)."""
    bm = bmesh.new()
    rings = []
    for p in prof:
        x, r = p[0], p[1]
        kk = p[2] if len(p) > 2 else 0.0
        if r <= 1e-9:
            rings.append([bm.verts.new((x, cy, cz))])
        else:
            rings.append([bm.verts.new((x + kk * r * math.sin(2 * math.pi * i / segs),
                                        cy + r * math.cos(2 * math.pi * i / segs),
                                        cz + r * math.sin(2 * math.pi * i / segs))) for i in range(segs)])
    for ra, rb in zip(rings[:-1], rings[1:]):
        for i in range(segs):
            j = (i + 1) % segs
            if len(ra) == 1 and len(rb) == 1:
                continue
            if len(ra) == 1:
                f = (ra[0], rb[j], rb[i])
            elif len(rb) == 1:
                f = (ra[i], ra[j], rb[0])
            else:
                f = (ra[i], ra[j], rb[j], rb[i])
            bm.faces.new(f)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return geo.obj_from_bm(name, bm, col, smooth=smooth)


def loft_x(name, stations, cy, cz, col, n=40):
    """Loft of 'stadium' sections (x, a, r): two half circles of radius r whose centres sit at cz +- a."""
    bm = bmesh.new()
    rings = []
    for x, a, r in stations:
        ring = []
        for i in range(n):
            th = 2 * math.pi * (i + 0.5) / n
            s = math.sin(th)
            ring.append(bm.verts.new((x, cy + r * math.cos(th), cz + r * s + (a if s > 0 else -a))))
        rings.append(ring)
    for ra, rb in zip(rings[:-1], rings[1:]):
        for i in range(n):
            j = (i + 1) % n
            bm.faces.new((ra[i], ra[j], rb[j], rb[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return geo.obj_from_bm(name, bm, col, smooth=50.0)


def fillet_path(P, rad, res=12):
    """Poly line -> list of (point, bend_id) with rounded corners (quadratic arcs); bend_id 0 = straight."""
    P = [Vector(p) for p in P]
    n = len(P)
    Q = [(P[0], 0)]
    for i in range(1, n - 1):
        a, b, c = P[i - 1], P[i], P[i + 1]
        d1, d2 = a - b, c - b
        l1, l2 = d1.length, d2.length
        d1.normalize()
        d2.normalize()
        ang = d1.angle(d2)
        t = min(rad / math.tan(ang / 2.0), l1 * (0.92 if i == 1 else 0.48), l2 * (0.92 if i == n - 2 else 0.48))
        p1, p2 = b + d1 * t, b + d2 * t
        for j in range(res + 1):
            u = j / res
            Q.append(((1 - u) ** 2 * p1 + 2 * u * (1 - u) * b + u ** 2 * p2, i))
    Q.append((P[-1], 0))
    return Q


# ----------------------------------------------------------------------------- part collector
class _Kit:
    def __init__(self, col):
        self.col = col
        self.groups = {}
        self._bolts = {}
        self._n = 0

    def nm(self, base):
        self._n += 1
        return "eng_%s_%03d" % (base, self._n)

    def add(self, ob, mat):
        geo.set_mat(ob, mats.get(mat))
        self.groups.setdefault(mat, []).append(ob)
        return ob

    def add_multi(self, ob, group):
        self.groups.setdefault(group, []).append(ob)
        return ob

    def lathe(self, mat, corners, pos, dirv, segs=64, res=3, closed=False, raw=False, smooth=40.0):
        prof = corners if raw else rprof(corners, res)
        ob = geo.lathe(self.nm("lat"), prof, segs, col=self.col, closed=closed, smooth=smooth)
        xf(ob, orient_m(pos, dirv))
        return self.add(ob, mat)

    def cyl(self, mat, r, p0, p1, segs=48, bevel=0.0, r2=None):
        p0, p1 = Vector(p0), Vector(p1)
        ob = geo.cyl(self.nm("cyl"), r, (p1 - p0).length, segs, bevel, col=self.col, r2=r2)
        xf(ob, orient_m((p0 + p1) / 2, p1 - p0))
        return self.add(ob, mat)

    def box(self, mat, size, loc, bevel=0.0, segs=2, m=None):
        ob = geo.box(self.nm("box"), size[0], size[1], size[2], bevel, segs, col=self.col, loc=loc)
        if m is not None:
            xf(ob, m)
        return self.add(ob, mat)

    def prism_xz(self, mat, outline, depth, bevel, bsegs=3, y=0.0):
        ob = geo.prism(self.nm("prism"), outline, depth, bevel=bevel, bsegs=bsegs, plane="XZ", col=self.col)
        if y:
            xf(ob, Matrix.Translation((0, y, 0)))
        return self.add(ob, mat)

    def plate(self, mat, outline, z0, z1, b=0.0, n=2, m=None, smooth=35.0):
        bm = bmesh.new()
        plate_bm(bm, outline, z0, z1, b, n)
        ob = geo.obj_from_bm(self.nm("plate"), bm, self.col, smooth=smooth)
        if m is not None:
            xf(ob, m)
        return self.add(ob, mat)

    def plate_xz(self, mat, outline, y0, y1, b=0.0, n=2):
        """Outline given in world (x, z), plate spans y0..y1."""
        return self.plate(mat, outline, -y1, -y0, b, n, RX90)

    def tube(self, mat, pts, r, fillet=0.0, segs=16, res=8):
        ob = geo.tube(self.nm("tube"), pts, r, segs=segs, fillet=fillet, fillet_res=res, col=self.col)
        return self.add(ob, mat)

    def torus(self, mat, R, r, pos, dirv, segs=40, rsegs=8):
        ob = geo.torus(self.nm("tor"), R, r, segs, rsegs, col=self.col)
        xf(ob, orient_m(pos, dirv))
        return self.add(ob, mat)

    def spring(self, mat, p0, p1, R, turns, wire, n=14):
        p0, p1 = Vector(p0), Vector(p1)
        ax = p1 - p0
        L = ax.length
        ax.normalize()
        ref = Vector((0, 0, 1)) if abs(ax.z) < 0.9 else Vector((1, 0, 0))
        e1 = ax.cross(ref).normalized()
        e2 = ax.cross(e1)
        N = int(turns * n)
        pts = []
        for i in range(N + 1):
            t = i / N
            a = 2 * math.pi * turns * t
            pts.append(p0 + ax * (L * t) + e1 * (R * math.cos(a)) + e2 * (R * math.sin(a)))
        ob = geo.tube(self.nm("spr"), pts, wire, segs=8, col=self.col)
        return self.add(ob, mat)

    def bolt(self, pos, n, d=0.006, kind="socket", mat=SP, washer=True, head_h=None):
        key = (round(d, 5), kind, washer, head_h)
        t = self._bolts.get(key)
        if t is None:
            t = geo.bolt(self.nm("bolt_t"), d, head_h, kind, col=self.col, washer=washer)
            self._bolts[key] = t
        ob = bpy.data.objects.new(self.nm("bolt"), t.data.copy())
        self.col.objects.link(ob)
        xf(ob, orient_m(pos, n))
        return self.add(ob, mat)

    def bolt_circle(self, c, n, rad, count, phase=0.0, **kw):
        m = orient_m(c, n)
        for i in range(count):
            a = phase + 2 * math.pi * i / count
            self.bolt(m @ Vector((rad * math.cos(a), rad * math.sin(a), 0)), n, **kw)

    def finish(self, root):
        for t in self._bolts.values():
            d = t.data
            bpy.data.objects.remove(t, do_unlink=True)
            if d.users == 0:
                bpy.data.meshes.remove(d)
        out = []
        for name, objs in self.groups.items():
            ob = geo.join(objs, "engine_" + name)
            out.append(ob)
        for ob in out:
            geo.parent(ob, root)
        return out


# ----------------------------------------------------------------------------- crankcase / gearbox
def _mount(k, x, ysurf, z):
    for s in (1, -1):
        k.cyl(BK, 0.014, (x, s * (ysurf - 0.004), z), (x, s * (ysurf + 0.008), z), bevel=0.0015)
        k.bolt((x, s * (ysurf + 0.008), z), (0, s, 0), d=0.010, kind="hex", head_h=0.006)


def _crankcase(k):
    prof = [(0.118, 0.258), (0.142, 0.295), (0.150, 0.345), (0.148, 0.400), (0.140, 0.447), (-0.016, 0.480),
            (-0.045, 0.490), (-0.150, 0.493), (-0.160, 0.500), (-0.172, 0.5405), (-0.203, 0.5405), (-0.214, 0.527),
            (-0.216, 0.495), (-0.232, 0.460), (-0.238, 0.400), (-0.232, 0.320), (-0.205, 0.275), (-0.150, 0.258)]
    k.prism_xz(BK, prof, 0.270, 0.007)
    # tilted horizontal parting flange through crank and output shaft
    k.prism_xz(BK, [(0.154, 0.3395), (0.154, 0.3455), (-0.242, 0.4068), (-0.242, 0.4008)], 0.278, 0.0015, 2)
    # sump + flange + side ribs
    k.prism_xz(BK, [(-0.135, 0.262), (-0.122, 0.203), (-0.105, 0.190), (0.100, 0.190), (0.118, 0.212), (0.118, 0.262)],
               0.210, 0.007)
    k.box(BK, (0.268, 0.226, 0.008), (-0.0085, 0, 0.258), bevel=0.002)
    for s in (1, -1):
        for i in range(4):
            k.box(BK, (0.170, 0.007, 0.0036), (0.015, s * 0.1055, 0.2035 + 0.012 * i), bevel=0.0012)
        for x in (-0.115, -0.04, 0.035, 0.105):          # sump pan screws
            k.bolt((x, s * 0.1095, 0.262), (0, 0, 1), d=0.005, mat="zinc")
    # drain plug (left side, low rear)
    k.cyl(BK, 0.016, (-0.09, 0.100, 0.222), (-0.09, 0.1048, 0.222), bevel=0.001)
    k.cyl("copper", 0.0135, (-0.09, 0.1045, 0.222), (-0.09, 0.1060, 0.222))
    k.cyl(SP, 0.0105, (-0.09, 0.1058, 0.222), (-0.09, 0.1130, 0.222), segs=6, bevel=0.001)
    # casting ribs on the front of the crankcase
    rib = [(0.132, 0.299), (0.140, 0.345), (0.138, 0.400), (0.130, 0.444), (0.141, 0.444), (0.1535, 0.400),
           (0.1555, 0.345), (0.143, 0.299)]
    for y in (-0.097, -0.045, 0.045, 0.097):
        k.prism_xz(BK, rib, 0.005, 0.0012, 2, y=y)
    # front lower mount: cast-in cross tube + bosses
    fx, fy, fz = MOUNTS["front_lower"]
    k.cyl(BK, 0.0195, (fx, -0.1345, fz), (fx, 0.1345, fz), bevel=0.003)
    _mount(k, fx, fy, fz)
    rx, ry, rz = MOUNTS["rear_upper"]
    _mount(k, rx, ry, rz)
    # ---- bevel drive housing (left rear) + output flange facing -X
    k.box(BK, (0.118, 0.093, 0.104), (-0.184, 0.1315, 0.405), bevel=0.016, segs=3)
    k.cyl(BK, 0.0475, (-0.2360, 0.13, 0.405), (-0.2455, 0.13, 0.405), bevel=0.0015, segs=64)
    k.lathe(AM, [(0.045, 0.0), (0.045, 0.0055, 0.0008), (0.038, 0.0055, 0.0005), (0.0375, 0.0015), (0.014, 0.0015),
                 (0.0138, 0.0055, 0.0004), (0.0092, 0.0055, 0.0006), (0.0088, -0.004), (0.0, -0.004)],
            (-0.2445, 0.13, 0.405), (-1, 0, 0), segs=72)
    k.bolt_circle((-0.2445 - 0.0015, 0.13, 0.405), (-1, 0, 0), 0.0265, 6, d=0.0055, head_h=0.003, washer=False)
    # bearing cap on the left face of the bevel box
    k.lathe(BK, [(0.0, 0.009), (0.022, 0.009, 0.003), (0.025, 0.004, 0.0015), (0.038, 0.004, 0.0015), (0.038, 0.0)],
            (-0.184, 0.1775, 0.405), (0, 1, 0))
    k.bolt_circle((-0.184, 0.1815, 0.405), (0, 1, 0), 0.0315, 4, phase=math.pi / 4, d=0.005)
    k.cyl(AM, 0.008, (-0.184, 0.186, 0.405), (-0.184, 0.1905, 0.405), segs=6, bevel=0.0008)


def _covers(k):
    ax, az = CRANK.x, CRANK.z
    # ---------------- left: alternator cover
    yl = 0.135
    k.lathe(BK, [(0.0, 0.046), (0.058, 0.046, 0.016), (0.066, 0.006, 0.003), (0.078, 0.006, 0.002), (0.078, 0.0)],
            (ax, yl, az), (0, 1, 0), segs=96)
    k.bolt_circle((ax, yl + 0.006, az), (0, 1, 0), 0.0722, 6, phase=math.radians(12), d=0.005)
    k.lathe(AM, [(0.0, 0.0038), (0.012, 0.0038, 0.0005), (0.0125, 0.0026, 0.0003), (0.0285, 0.0026, 0.0008),
                 (0.030, 0.0)], (ax, yl + 0.046, az), (0, 1, 0), segs=72)
    k.bolt_circle((ax, yl + 0.0486, az), (0, 1, 0), 0.0215, 3, phase=math.radians(90), d=0.004, kind="button",
                  washer=False, mat=SD)
    # left rear (shift / starter gear) cover
    lr = round_poly([(-0.052, 0.292), (-0.042, 0.330), (-0.042, 0.435), (-0.060, 0.472), (-0.135, 0.472),
                     (-0.135, 0.292)], 0.012)
    k.plate_xz(BK, lr, 0.131, 0.151, 0.004, 3)
    for x, z in ((-0.058, 0.303), (-0.049, 0.385), (-0.062, 0.459), (-0.105, 0.464), (-0.105, 0.300)):
        k.bolt((x, 0.151, z), (0, 1, 0), d=0.005, mat="zinc")
    # gear shift shaft stub + seal
    k.cyl(BK, 0.013, (-0.078, 0.150, 0.338), (-0.078, 0.1545, 0.338), bevel=0.001)
    k.cyl("rubber", 0.0095, (-0.078, 0.1540, 0.338), (-0.078, 0.1560, 0.338))
    k.cyl(SP, 0.006, (-0.078, 0.155, 0.338), (-0.078, 0.176, 0.338), segs=24, bevel=0.0008)
    # ---------------- right: outer cover (hull of crank + clutch circles), clutch dome, ignition cover
    bx, bz = -0.065, 0.380
    yr = -0.149
    hull = hull_outline([(ax, az, 0.068), (bx, bz, 0.104)])
    k.plate_xz(BK, hull, yr, -0.131, 0.004, 3)
    for x, z in along_outline(hull, 12, 0.0068, phase=0.3):
        k.bolt((x, yr, z), (0, -1, 0), d=0.005)
    k.lathe(BK, [(0.0, 0.036), (0.052, 0.036, 0.012), (0.075, 0.021, 0.007), (0.079, 0.005, 0.003), (0.087, 0.0)],
            (bx, yr, bz), (0, -1, 0), segs=96)
    k.lathe(AM, [(0.0, 0.0042), (0.0265, 0.0042, 0.001), (0.029, 0.0)], (bx, yr - 0.036, bz), (0, -1, 0), segs=72)
    k.bolt_circle((bx, yr - 0.0402, bz), (0, -1, 0), 0.0205, 3, phase=math.radians(30), d=0.004, kind="button",
                  washer=False, mat=SD)
    # ignition / timing cover: machined bright accent
    k.lathe(AM, [(0.0, 0.0155), (0.020, 0.0155, 0.0006), (0.0205, 0.0140, 0.0004), (0.036, 0.0140, 0.004),
                 (0.040, 0.004, 0.002), (0.052, 0.004, 0.0012), (0.052, 0.0)], (ax, yr, az), (0, -1, 0), segs=96)
    k.bolt_circle((ax, yr - 0.004, az), (0, -1, 0), 0.046, 4, phase=math.radians(45), d=0.005, mat=SD)
    # small round cap + casting ribs on the right rear of the gearbox
    k.lathe(BK, [(0.0, 0.007), (0.013, 0.007, 0.002), (0.016, 0.003, 0.0012), (0.021, 0.003, 0.0012), (0.021, 0.0)],
            (-0.196, -0.135, 0.318), (0, -1, 0), segs=48)
    k.cyl(AM, 0.0075, (-0.196, -0.1415, 0.318), (-0.196, -0.1465, 0.318), segs=6, bevel=0.0008)
    for z in (0.398, 0.418, 0.438):
        k.box(BK, (0.046, 0.007, 0.0042), (-0.201, -0.1352, z), bevel=0.0014)
    rib = [(-0.208, 0.492), (-0.224, 0.460), (-0.230, 0.400), (-0.224, 0.322), (-0.199, 0.280), (-0.2085, 0.2745),
           (-0.2365, 0.319), (-0.2430, 0.400), (-0.2365, 0.461), (-0.2200, 0.495)]
    for y in (-0.095, -0.040, 0.015):
        k.prism_xz(BK, rib, 0.005, 0.0012, 2, y=y)
    # oil filler cap (top of the gearbox, right) and breather stub
    k.cyl(BK, 0.018, (-0.085, -0.105, 0.488), (-0.085, -0.105, 0.4955), bevel=0.001)
    k.lathe(AM, [(0.0, 0.0125), (0.006, 0.0125, 0.0005), (0.0065, 0.0105, 0.0004), (0.0125, 0.0105, 0.0015),
                 (0.0155, 0.0085, 0.001), (0.0155, 0.0025, 0.0006), (0.013, 0.002), (0.013, 0.0)],
            (-0.085, -0.105, 0.4955), (0, 0, 1), segs=48)
    k.cyl(SP, 0.009, (-0.130, -0.118, 0.4925), (-0.130, -0.118, 0.5005), segs=6, bevel=0.0008)
    k.cyl(SP, 0.0042, (-0.130, -0.118, 0.500), (-0.130, -0.118, 0.514), segs=20)


# ----------------------------------------------------------------------------- cylinders, head, cam cover
def _cylinders(k):
    # base flange, barrels, cam chain tunnel
    k.plate(BK, rrect(0.078, 0.152, 0.02), 0.100, 0.113, 0.003, 3, M_CYL)
    for s in (1, -1):
        k.cyl(BK, 0.060, Lp(0, s * 0.075, 0.105), Lp(0, s * 0.075, 0.188), segs=64)
    k.box(BK, (0.090, 0.080, 0.083), (0, 0, 0.1465), m=M_CYL)
    # joint plates (block deck / head deck) and head core
    k.plate(BK, rrect(0.073, 0.150, 0.028, 8), 0.1845, 0.1899, 0.001, 2, M_CYL)
    k.plate(BK, rrect(0.074, 0.151, 0.028, 8), 0.1901, 0.1955, 0.001, 2, M_CYL)
    k.plate(BK, rrect(0.060, 0.137, 0.03, 8), 0.190, 0.250, 0.0, 2, M_CYL)
    # fins: one mesh, real geometry
    bm = bmesh.new()
    ob_ = fin_outline(0.083, 0.083, 0.162, 0.028)
    for i in range(9):
        d0 = 0.1195 + 0.0072 * i
        plate_bm(bm, ob_, d0, d0 + 0.0021, 0.0005, 1)
    oh_ = fin_outline(0.088, 0.080, 0.165, 0.030)
    for i in range(7):
        d0 = 0.1975 + 0.0072 * i
        plate_bm(bm, oh_, d0, d0 + 0.0021, 0.0005, 1)
    fins = geo.obj_from_bm(k.nm("fins"), bm, k.col, smooth=30.0)
    xf(fins, M_CYL)
    k.add(fins, "engine_fin")
    # head top box (cam box) with the head mounts on its sides
    k.plate(BK, rrect(0.075, 0.156, 0.012, 5), 0.249, 0.278, 0.004, 3, M_CYL)
    hx, hy, hz = MOUNTS["head"]
    _mount(k, hx, hy, hz)
    for su in (1, -1):
        for sy in (1, -1):
            k.bolt(Lp(su * 0.055, sy * 0.147, 0.278), Ld(0, 0, 1), d=0.007, kind="hex")
    # ports
    for s in (1, -1):
        k.cyl(BK, 0.027, Lp(0.040, s * 0.075, 0.219), Lp(0.0895, s * 0.075, 0.219), bevel=0.002)      # exhaust
        k.lathe(AM, [(0.0195, 0.0), (0.033, 0.0, 0.0006), (0.033, 0.003, 0.0006), (0.0275, 0.003), (0.0275, 0.0048),
                     (0.0315, 0.0048, 0.0005), (0.0315, 0.0068, 0.0005), (0.0275, 0.0068), (0.0275, 0.0086),
                     (0.0300, 0.0086, 0.0005), (0.0300, 0.0106, 0.0005), (0.0195, 0.0106)],
                Lp(0.0885, s * 0.075, 0.219), Ld(1, 0, 0), segs=64)
        k.cyl(BK, 0.027, Lp(-0.040, s * 0.075, 0.222), Lp(-0.086, s * 0.075, 0.222), bevel=0.002)      # intake
    # ---- cam cover
    k.plate(BK, rrect(0.060, 0.128, 0.010, 5), 0.2775, 0.2845, 0.002, 2, M_CYL)
    for su in (1, -1):
        k.cyl(BK, 0.021, Lp(su * 0.031, -0.120, 0.281), Lp(su * 0.031, 0.120, 0.281), segs=64, bevel=0.003)
        for sy in (1, -1):
            k.lathe(AM, [(0.0, 0.0030), (0.0045, 0.0030, 0.0004), (0.005, 0.0042, 0.0003), (0.0145, 0.0042, 0.001),
                         (0.0165, 0.0)], Lp(su * 0.031, sy * 0.1198, 0.281), (0, sy, 0), segs=48)
            for y in (0.040, 0.112):
                k.bolt(Lp(su * 0.0562, sy * y, 0.2845), Ld(0, 0, 1), d=0.0045)
    k.box(BK, (0.062, 0.232, 0.0115), (0, 0, 0.2898), bevel=0.002, m=M_CYL)
    top = k.plate("alu_brushed", rrect(0.0195, 0.100, 0.004, 4), 0.2950, 0.2982, 0.0008, 2, M_CYL)
    for y in (-0.085, -0.03, 0.03, 0.085):
        k.bolt(Lp(0, y, 0.2982), Ld(0, 0, 1), d=0.0045, kind="button", washer=False)


# ----------------------------------------------------------------------------- intake
def _intake(k):
    back = Ld(-1, 0, 0)
    d0 = 0.222
    for s in (1, -1):
        y = s * 0.075
        # rubber boot + clamps
        k.lathe("rubber", [(0.026, 0.0), (0.0305, 0.0, 0.002), (0.0305, 0.022, 0.002), (0.026, 0.022)],
                Lp(-0.083, y, d0), back, segs=48)
        for h in (0.0035, 0.0145):
            k.lathe(SP, [(0.0303, 0.0), (0.0314, 0.0), (0.0314, 0.005), (0.0303, 0.005)], Lp(-0.083 - h, y, d0), back,
                    segs=48, raw=True, closed=True)
            k.cyl(SP, 0.003, Lp(-0.0855 - h, y + s * 0.012, d0 + 0.0295), Lp(-0.0855 - h, y - s * 0.004, d0 + 0.0325),
                  segs=12)
        # throttle body
        k.lathe("alu_cast", [(0.024, 0.0), (0.0285, 0.0, 0.001), (0.0285, 0.048, 0.001), (0.024, 0.048)],
                Lp(-0.104, y, d0), back, segs=64)
        k.cyl("alu_cast", 0.008, Lp(-0.127, y - 0.034, d0), Lp(-0.127, y + 0.034, d0), segs=32, bevel=0.001)
        k.cyl("alu_cast", 0.010, Lp(-0.116, y, d0 + 0.020), Lp(-0.1195, y, d0 + 0.0315), segs=32, bevel=0.001)
        # injector
        k.cyl("plastic_black", 0.0068, Lp(-0.118, y, d0 + 0.026), Lp(-0.1262, y, d0 + 0.048), segs=24, bevel=0.001)
        k.cyl("plastic_black", 0.0085, Lp(-0.1250, y, d0 + 0.0445), Lp(-0.1285, y, d0 + 0.0540), segs=24, bevel=0.0012)
        # butterfly
        k.cyl(SP, 0.0255, Lp(-0.1268, y, d0 - 0.0015), Lp(-0.1272, y, d0 + 0.0015), segs=48)
        # velocity stack
        prof = [(0.0288, 0.0)]
        for i in range(11):
            t = i / 10
            prof.append((0.0288 + 0.0112 * t ** 2.4, 0.010 + 0.036 * t))
        prof += [(0.0412, 0.0474), (0.0408, 0.0488), (0.0394, 0.0493), (0.0380, 0.0482)]
        for i in range(10, -1, -1):
            t = i / 10
            prof.append((0.0266 + 0.0112 * t ** 2.4, 0.010 + 0.036 * t))
        prof.append((0.0262, -0.022))
        k.lathe(AM, prof, Lp(-0.150, y, d0), back, segs=72, raw=True, smooth=50.0)
        k.lathe(AM, [(0.0290, 0.0), (0.0302, 0.0), (0.0302, 0.005), (0.0290, 0.005)], Lp(-0.151, y, d0), back,
                segs=64, raw=True, closed=True)
    # linkage shaft, pulley, fuel rail
    k.cyl(SP, 0.0032, Lp(-0.127, -0.112, d0), Lp(-0.127, 0.112, d0), segs=16)
    k.lathe(AM, [(0.0, 0.004), (0.006, 0.004), (0.0065, 0.0032), (0.016, 0.0032, 0.0005), (0.017, 0.0022),
                 (0.0145, 0.0), (0.017, -0.0022), (0.016, -0.0032, 0.0005), (0.0065, -0.0032), (0.006, -0.004),
                 (0.0, -0.004)], Lp(-0.127, 0.0, d0), (0, 1, 0), segs=48)
    k.cyl("ano_black", 0.0042, Lp(-0.1270, -0.086, d0 + 0.050), Lp(-0.1270, 0.086, d0 + 0.050), segs=24, bevel=0.001)
    k.cyl(SP, 0.0052, Lp(-0.1270, -0.004, d0 + 0.050), Lp(-0.1270, 0.004, d0 + 0.050), segs=6)


# ----------------------------------------------------------------------------- ignition
def _ignition(k):
    a = math.radians(25.0)
    ax = Vector((math.sin(a), 0.0, math.cos(a)))          # plug axis in cylinder-local coords
    wdir = (R_CYL @ ax).normalized()
    for s in (1, -1):
        base = Vector((0.066, s * 0.075, 0.2775))

        def P(t):
            return M_CYL @ (base + ax * t)
        k.cyl(BK, 0.0115, P(-0.003), P(0.0015), segs=32, bevel=0.0008)
        k.cyl(SP, 0.0085, P(0.001), P(0.007), segs=6, bevel=0.0006)
        k.cyl("white_paint", 0.0045, P(0.006), P(0.016), segs=20)
        k.lathe("rubber", [(0.0075, 0.0), (0.0086, 0.0015), (0.0086, 0.024, 0.003), (0.0078, 0.031, 0.003), (0.0, 0.031)],
                P(0.011), wdir, segs=32)
        p0 = base + ax * 0.035
        pts = [p0, Vector((p0.x, s * 0.1325, p0.z)), Vector((0.045, s * 0.1325, 0.2825)),
               Vector((-0.055, s * 0.1325, 0.2825)), Vector((-0.089, s * 0.128, 0.272)), Vector((-0.089, s * 0.043, 0.261))]
        k.tube("rubber", [M_CYL @ p for p in pts], 0.0035, fillet=0.014, segs=12, res=8)
        k.tube("rubber", [M_CYL @ p0, M_CYL @ Vector((p0.x, s * 0.096, p0.z))], 0.0058, segs=16)      # elbow boot
        # coil tower boot
        k.cyl("rubber", 0.0068, Lp(-0.089, s * 0.036, 0.261), Lp(-0.089, s * 0.050, 0.261), segs=20, bevel=0.001)
        # lead clip on the head deck
        k.box(SP, (0.006, 0.011, 0.0012), (-0.02, s * 0.1325, 0.2866), m=M_CYL)
        k.bolt(Lp(-0.02, s * 0.139, 0.278), Ld(0, 0, 1), d=0.0035, kind="button", washer=False)
    # coil pack behind the head, between the intake runners
    k.box("plastic_black", (0.030, 0.074, 0.030), (-0.089, 0.0, 0.261), bevel=0.003, m=M_CYL)
    for y in (-0.022, 0.022):
        k.bolt(Lp(-0.089, y, 0.276), Ld(0, 0, 1), d=0.004, mat=SD)


# ----------------------------------------------------------------------------- starter, filter, lines
def _ancillaries(k):
    # starter motor (transverse, on top of the gearbox behind the block) + gear housing on the left
    sx, sz = -0.085, 0.517
    k.cyl(BK, 0.031, (sx, -0.035, sz), (sx, 0.106, sz), segs=64, bevel=0.003)
    k.cyl(BK, 0.037, (sx, 0.103, sz), (sx, 0.135, sz), segs=64, bevel=0.004)
    k.lathe("plastic_black", [(0.0, 0.011), (0.018, 0.011, 0.003), (0.029, 0.006, 0.003), (0.0305, 0.0)],
            (sx, -0.035, sz), (0, -1, 0), segs=64)
    for dz in (-0.021, 0.021):
        k.bolt((sx, -0.0415, sz + dz), (0, -1, 0), d=0.004, kind="hex", washer=False)
    k.lathe(SP, [(0.0306, 0.0), (0.0318, 0.0), (0.0318, 0.008), (0.0306, 0.008)], (sx, 0.030, sz), (0, 1, 0),
            segs=64, raw=True, closed=True)
    k.bolt_circle((sx, 0.135, sz), (0, 1, 0), 0.029, 3, phase=math.radians(90), d=0.005)
    # starter terminal + rubber boot + cable going down behind the case
    k.cyl(SP, 0.003, (sx - 0.02, -0.020, sz + 0.020), (sx - 0.027, -0.020, sz + 0.029), segs=12)
    k.tube("rubber", [(sx - 0.024, -0.020, sz + 0.025), (sx - 0.040, -0.020, sz + 0.045), (-0.150, -0.020, 0.560),
                      (-0.180, -0.020, 0.530), (-0.180, -0.020, 0.500)], 0.0045, fillet=0.02, segs=12)
    # oil filter cartridge, front low
    fz = 0.277
    k.cyl(BK, 0.035, (0.112, 0, fz), (0.1375, 0, fz), segs=64, bevel=0.002)
    k.lathe("ano_black", [(0.0312, 0.0), (0.0312, 0.041, 0.004), (0.024, 0.0465, 0.002), (0.0, 0.0465)],
            (0.1365, 0, fz), (1, 0, 0), segs=64)
    k.lathe(AM, [(0.0312, 0.0), (0.0324, 0.0), (0.0324, 0.005), (0.0312, 0.005)], (0.1385, 0, fz), (1, 0, 0),
            segs=64, raw=True, closed=True)
    k.cyl(AM, 0.012, (0.1825, 0, fz), (0.1895, 0, fz), segs=6, bevel=0.0008)
    # external oil feed lines (hard, polished) from the case top up to the head, one per side
    bw = Lp(-0.092, 0, 0.112)
    for s in (1, -1):
        A = Vector((bw.x, s * 0.118, 0.4875))
        k.cyl(SP, 0.0075, A - Vector((0, 0, 0.0045)), A + Vector((0, 0, 0.0035)), segs=24, bevel=0.0008)
        k.bolt(A + Vector((0, 0, 0.0035)), (0, 0, 1), d=0.006, kind="hex", washer=False, head_h=0.004)
        E = Lp(-0.05, s * 0.1595, 0.262)
        pts = [A, Vector((bw.x, s * 0.172, 0.4875)), Lp(-0.05, s * 0.172, 0.160), Lp(-0.05, s * 0.172, 0.262), E]
        k.tube("braid", pts, 0.0042, fillet=0.012, segs=16, res=8)
        k.cyl("ano_blue", 0.0066, A + Vector((0, s * 0.0085, 0)), A + Vector((0, s * 0.0165, 0)), segs=6, bevel=0.0006)
        k.cyl(AM, 0.0054, A + Vector((0, s * 0.0165, 0)), A + Vector((0, s * 0.0255, 0)), segs=24, bevel=0.0006)
        k.cyl("ano_blue", 0.0066, Lp(-0.05, s * 0.172, 0.236), Lp(-0.05, s * 0.172, 0.244), segs=6, bevel=0.0006)
        k.cyl(AM, 0.0054, Lp(-0.05, s * 0.172, 0.226), Lp(-0.05, s * 0.172, 0.236), segs=24, bevel=0.0006)
        k.box("zinc", (0.008, 0.0015, 0.011), (-0.05, s * 0.1672, 0.200), m=M_CYL)
        k.cyl(SP, 0.0075, Vector((E.x, s * 0.1555, E.z)), Vector((E.x, s * 0.1630, E.z)), segs=24, bevel=0.0008)
        k.bolt(Vector((E.x, s * 0.1630, E.z)), (0, s, 0), d=0.006, kind="hex", washer=False, head_h=0.004)
    # breather: nipple on the cam cover, hose down to the stub on the gearbox top (right side)
    k.cyl(SP, 0.0042, Lp(-0.046, -0.105, 0.284), Lp(-0.064, -0.105, 0.284), segs=16)
    hose = [Lp(-0.058, -0.105, 0.284), Lp(-0.100, -0.108, 0.282), Vector((-0.050, -0.127, 0.585)),
            Vector((-0.130, -0.127, 0.570)), Vector((-0.130, -0.118, 0.506))]
    k.tube("rubber", hose, 0.0055, fillet=0.025, segs=16, res=10)
    k.lathe(SP, [(0.0056, 0.0), (0.0064, 0.0), (0.0064, 0.004), (0.0056, 0.004)], (-0.130, -0.118, 0.507), (0, 0, 1),
            segs=24, raw=True, closed=True)


# ----------------------------------------------------------------------------- exhaust
def _header(k, P, r, side):
    Q = fillet_path(P, 0.050, 12)
    pts = [q for q, _ in Q]
    ob = geo.tube(k.nm("hdr"), pts, r, segs=24, col=k.col)
    s = [0.0]
    for i in range(1, len(pts)):
        s.append(s[-1] + (pts[i] - pts[i - 1]).length)
    ids = [b for _, b in Q]
    first_end = max(s[i] for i in range(len(pts)) if ids[i] == 1)
    seg_tag = [1 if ((ids[i] and ids[i] == ids[i + 1]) or s[i + 1] <= first_end + 1e-6) else 0
               for i in range(len(pts) - 1)]
    mids = [(pts[i] + pts[i + 1]) * 0.5 for i in range(len(pts) - 1)]
    me = ob.data
    me.materials.clear()
    me.materials.append(mats.get("steel_brushed"))
    me.materials.append(mats.get("ti_blued"))
    for poly in me.polygons:
        c = poly.center
        j = min(range(len(mids)), key=lambda i: (mids[i] - c).length_squared)
        poly.material_index = seg_tag[j]
    k.add_multi(ob, "headers")
    # weld beads where the sections meet, slip-joint spring at the first joint
    sprung = False
    for i in range(len(seg_tag) - 1):
        if seg_tag[i] != seg_tag[i + 1]:
            k.torus("steel_brushed", r + 0.0003, 0.0011, pts[i + 1], pts[i + 2] - pts[i], segs=32, rsegs=8)
            if not sprung:
                sprung = True
                c = pts[i + 1]
                t = (pts[i + 2] - pts[i]).normalized()
                sd = Vector((0, side, 0))
                sd = (sd - t * sd.dot(t)).normalized()
                for sg_ in (1.0, -1.0):
                    o = sd * (sg_ * (r + 0.0032))
                    k.spring("steel_brushed", c - t * 0.015 + o, c + t * 0.015 + o, 0.0022, 11, 0.00055)
                    for q in (-0.0165, 0.0165):
                        k.cyl("steel_brushed", 0.0012, c + t * q + sd * (sg_ * (r - 0.001)),
                              c + t * q + sd * (sg_ * (r + 0.0046)), segs=8)


def _exhaust(k):
    r = 0.0185
    yc, zc = -0.255, 0.205
    z_up, z_lo = 0.224, 0.186
    du = Ld(1, 0, 0)
    for s in (-1, 1):
        H0 = Lp(0.090, s * 0.075, 0.219)
        H1 = H0 + du * ((0.240 - H0.x) / du.x)
        if s < 0:
            P = [H0, H1, (0.197, -0.085, 0.300), (0.218, yc, z_up), (0.135, yc, z_up)]
        else:
            P = [H0, H1, (0.196, 0.075, 0.212), (0.218, yc, z_lo), (0.135, yc, z_lo)]
        _header(k, P, r, s)
    # 2-into-1 collector + link pipe
    a0 = (z_up - z_lo) / 2
    st = [(0.1605, a0, 0.0187), (0.1590, a0, 0.0200), (0.1575, a0, 0.0212), (0.136, a0, 0.0212)]
    for i in range(1, 11):
        t = i / 10
        e = t * t * (3 - 2 * t)
        st.append((0.136 - 0.056 * t, a0 * (1 - e), 0.0212 + (0.027 - 0.0212) * e))
    st.append((0.028, 0.0, 0.027))
    k.add(loft_x(k.nm("collector"), st, yc, zc, k.col, n=48), "steel_brushed")
    k.torus("steel_brushed", 0.0272, 0.0012, (0.080, yc, zc), (1, 0, 0), segs=48)
    # clamp on the link pipe
    k.add(lathe_x(k.nm("clamp"), [(0.052, 0.0268), (0.052, 0.0284), (0.0513, 0.0291), (0.0387, 0.0291),
                                  (0.038, 0.0284), (0.038, 0.0268)], yc, zc, k.col, segs=64), SP)
    k.box(SP, (0.012, 0.012, 0.008), (0.045, yc - 0.004, zc + 0.0315), bevel=0.0015)
    k.bolt((0.045, yc + 0.002, zc + 0.0315), (0, 1, 0), d=0.005, kind="hex", washer=False, mat=SD)
    # ---- muffler (lathe along X)
    R = 0.048
    ti = "titanium"
    k.add(lathe_x(k.nm("can_in"), [(0.0315, 0.0272), (0.0320, 0.0285), (0.0315, 0.0296), (0.0190, 0.0296),
                                   (0.0165, 0.0305), (-0.0095, 0.0470), (-0.0120, R), (-0.0140, R)],
                  yc, zc, k.col, segs=72), ti)
    k.add(lathe_x(k.nm("can_blue"), [(-0.0140, R), (-0.0480, R)], yc, zc, k.col, segs=72), "ti_blued")
    k.add(lathe_x(k.nm("can_body"), [(-0.0480, R), (-0.2620, R)], yc, zc, k.col, segs=72), ti)
    for x in (-0.0140, -0.0480):
        k.torus(ti, R, 0.0010, (x, yc, zc), (1, 0, 0), segs=72, rsegs=8)
    sl = -0.25                                              # slash: top reaches further back
    k.add(lathe_x(k.nm("can_tail"), [(-0.2620, R), (-0.2625, 0.0492), (-0.2632, 0.0497), (-0.2752, 0.0497),
                                     (-0.2760, 0.0490), (-0.2765, 0.0462), (-0.2970, 0.0462, sl),
                                     (-0.2982, 0.0455, sl), (-0.2986, 0.0440, sl), (-0.2982, 0.0425, sl),
                                     (-0.2970, 0.0418, sl)], yc, zc, k.col, segs=72), ti)
    k.add(lathe_x(k.nm("can_inner"), [(-0.2970, 0.0418, sl), (-0.2620, 0.0418), (-0.2620, 0.0290),
                                      (-0.2890, 0.0290), (-0.2900, 0.0278), (-0.2890, 0.0266), (-0.2000, 0.0266),
                                      (-0.2000, 0.0)], yc, zc, k.col, segs=72), SD)
    # rivets on both end caps
    for x, cnt in ((-0.0200, 8), (-0.2560, 8)):
        for i in range(cnt):
            a = 2 * math.pi * (i + 0.5) / cnt
            n = Vector((0, math.cos(a), math.sin(a)))
            k.bolt(Vector((x, yc, zc)) + n * R, n, d=0.004, kind="button", washer=False)
    # strap + hanger bracket
    k.add(lathe_x(k.nm("strap"), [(-0.1590, R), (-0.1592, 0.0492), (-0.1600, 0.0498), (-0.1760, 0.0498),
                                  (-0.1768, 0.0492), (-0.1770, R)], yc, zc, k.col, segs=72), SP)
    hang = round_poly([(-0.181, 0.2520), (-0.155, 0.2520), (-0.157, 0.297), (-0.1605, 0.3065), (-0.168, 0.3100),
                       (-0.1755, 0.3065), (-0.179, 0.297)], 0.004)
    k.plate_xz("alu_brushed", hang, yc - 0.0025, yc + 0.0025, 0.001, 2)
    k.cyl("rubber", 0.0088, (-0.168, yc - 0.0065, 0.298), (-0.168, yc + 0.0065, 0.298), segs=32, bevel=0.001)
    k.bolt((-0.168, yc - 0.0065, 0.298), (0, -1, 0), d=0.006, kind="hex")
    for x in (-0.174, -0.162):
        k.bolt((x, yc - 0.0025, 0.2625), (0, -1, 0), d=0.0035, kind="button", washer=False)


# ============================================================================= detail pass 2
def _sensor(k, c, n, wire=None):
    """Hex sensor plug on a cast boss with a rubber boot (c on the casting surface, n outward)."""
    c, n = Vector(c), Vector(n).normalized()
    k.cyl(BK, 0.010, c - n * 0.002, c + n * 0.0032, bevel=0.001, segs=32)
    k.cyl("zinc", 0.0070, c + n * 0.0030, c + n * 0.0075, segs=6, bevel=0.0006)
    k.cyl("rubber", 0.0058, c + n * 0.0073, c + n * 0.0185, segs=20, r2=0.0038, bevel=0.001)
    if wire:
        k.tube("rubber", wire, 0.002, fillet=0.008, segs=8, res=6)


def _detail_cases(k):
    Z = "zinc"
    # ---- right rear of the gearbox: oil sight glass (brass ring), machined ID pad, casting marks
    sg = Vector((-0.200, -0.135, 0.362))
    k.lathe(BK, [(0.0, 0.002), (0.0145, 0.002, 0.001), (0.0158, 0.0)], sg, (0, -1, 0), segs=48)
    k.lathe("brass", [(0.0078, 0.0010), (0.0082, 0.0042, 0.0005), (0.0112, 0.0042, 0.0008), (0.0118, 0.0)],
            sg + Vector((0, -0.002, 0)), (0, -1, 0), segs=48)
    k.cyl("amber_fluid", 0.0080, sg + Vector((0, -0.0021, 0)), sg + Vector((0, -0.0036, 0)), segs=32)
    k.cyl("glass", 0.0080, sg + Vector((0, -0.0038, 0)), sg + Vector((0, -0.0054, 0)), segs=32)
    k.box(BK, (0.032, 0.003, 0.013), (-0.200, -0.1355, 0.472), bevel=0.001)
    k.box(AM, (0.028, 0.0012, 0.009), (-0.200, -0.1373, 0.472), bevel=0.0004)
    for x in (-0.176, -0.168, -0.160):
        k.box(BK, (0.005, 0.0026, 0.0022), (x, -0.1353, 0.288), bevel=0.0006)
    for i in range(4):
        x = -0.150 + 0.006 * i
        k.cyl(BK, 0.0014, (x, -0.1345, 0.288), (x, -0.1365, 0.288), segs=12)
    # ---- rear face: plug with copper washer, casting marks
    k.cyl(BK, 0.012, (-0.232, -0.068, 0.36), (-0.2400, -0.068, 0.36), bevel=0.001)
    k.cyl("copper", 0.0095, (-0.2398, -0.068, 0.36), (-0.2408, -0.068, 0.36), segs=24)
    k.cyl(Z, 0.0075, (-0.2406, -0.068, 0.36), (-0.2458, -0.068, 0.36), segs=6, bevel=0.0006)
    for i in range(3):
        k.box(BK, (0.003, 0.006, 0.0022), (-0.2342, -0.078 + 0.009 * i, 0.442), bevel=0.0006)
    for i in range(4):
        k.cyl(BK, 0.0014, (-0.2330, -0.079 + 0.006 * i, 0.432), (-0.2362, -0.079 + 0.006 * i, 0.432), segs=12)
    # ---- gearbox top plug, front machined pad
    k.cyl(BK, 0.011, (-0.129, 0.035, 0.490), (-0.129, 0.035, 0.4965), bevel=0.001)
    k.cyl(Z, 0.007, (-0.129, 0.035, 0.4962), (-0.129, 0.035, 0.5015), segs=6, bevel=0.0006)
    k.box(BK, (0.004, 0.036, 0.016), (0.1475, 0, 0.398), bevel=0.001)
    k.box(AM, (0.0012, 0.031, 0.011), (0.1500, 0, 0.398), bevel=0.0004)
    # ---- sump oil temperature sensor (front right) with boot and wire stub
    _sensor(k, (0.1165, -0.05, 0.236), (1, 0, 0),
            [(0.133, -0.05, 0.236), (0.156, -0.05, 0.240), (0.158, -0.068, 0.275), (0.149, -0.070, 0.300),
             (0.1535, -0.070, 0.336)])
    k.box(Z, (0.0015, 0.008, 0.005), (0.1535, -0.070, 0.318))
    # ---- crank sensor (right, front upper) with wire clipped along the case
    _sensor(k, (0.105, -0.135, 0.415), (0, -1, 0),
            [(0.105, -0.1525, 0.415), (0.105, -0.156, 0.428), (0.098, -0.139, 0.441), (0.03, -0.139, 0.452),
             (-0.015, -0.139, 0.462), (-0.036, -0.138, 0.480), (-0.040, -0.127, 0.4955), (-0.050, -0.122, 0.4965)])
    for x, z in ((0.07, 0.4455), (0.01, 0.4565)):
        k.box(Z, (0.006, 0.0014, 0.0075), (x, -0.1416, z + 0.001))
        k.bolt((x, -0.135, z + 0.0075), (0, -1, 0), d=0.0035, kind="button", washer=False, mat=Z)
    k.box("plastic_black", (0.014, 0.008, 0.007), (-0.056, -0.122, 0.4965), bevel=0.0015)
    # ---- gear position sensor (left rear cover) with wire up to the loom
    _sensor(k, (-0.112, 0.151, 0.345), (0, 1, 0),
            [(-0.112, 0.168, 0.345), (-0.112, 0.172, 0.358), (-0.118, 0.156, 0.372), (-0.118, 0.1545, 0.462),
             (-0.128, 0.150, 0.480), (-0.148, 0.126, 0.4975)])
    k.box(Z, (0.007, 0.0014, 0.006), (-0.118, 0.1568, 0.420))
    k.bolt((-0.124, 0.151, 0.420), (0, 1, 0), d=0.0035, kind="button", washer=False, mat=Z)
    for i in range(3):
        k.box(BK, (0.005, 0.0026, 0.0022), (-0.098 + 0.008 * i, 0.1512, 0.418), bevel=0.0006)
    for i in range(5):
        x = -0.099 + 0.006 * i
        k.cyl(BK, 0.0014, (x, 0.1505, 0.408), (x, 0.1525, 0.408), segs=12)
    # ---- cam chain tunnel, tensioner, tappet caps, extra head nuts
    k.box(BK, (0.146, 0.040, 0.146), (0, 0, 0.177), bevel=0.004, m=M_CYL)
    k.cyl(BK, 0.012, Lp(-0.060, 0, 0.150), Lp(-0.0875, 0, 0.150), bevel=0.0015, segs=32)
    k.box(BK, (0.004, 0.048, 0.018), (-0.0855, 0, 0.150), bevel=0.0015, m=M_CYL)
    k.cyl(AM, 0.008, Lp(-0.0875, 0, 0.150), Lp(-0.0945, 0, 0.150), segs=6, bevel=0.0006)
    for y in (-0.018, 0.018):
        k.bolt(Lp(-0.0875, y, 0.150), Ld(-1, 0, 0), d=0.004, mat=Z)
    for su in (1, -1):
        for sy in (1, -1):
            k.lathe(AM, [(0.0, 0.0024), (0.0092, 0.0024, 0.0008), (0.0105, 0.0)], Lp(su * 0.031, sy * 0.075, 0.3011),
                    Ld(0, 0, 1), segs=40)
            k.cyl(AM, 0.0045, Lp(su * 0.031, sy * 0.075, 0.3033), Lp(su * 0.031, sy * 0.075, 0.3048), segs=6)
    for u, y in ((0.0675, 0.0), (-0.0675, 0.0), (-0.0675, 0.075), (-0.0675, -0.075), (0.0675, 0.118), (0.0675, -0.118)):
        k.bolt(Lp(u, y, 0.278), Ld(0, 0, 1), d=0.0065, kind="hex", mat=Z)
    # ---- oil filler cap: knurl + safety wire to the cable lug
    for i in range(22):
        a = 2 * math.pi * i / 22
        x, y = -0.085 + 0.0156 * math.cos(a), -0.105 + 0.0156 * math.sin(a)
        k.cyl(AM, 0.0011, (x, y, 0.4985), (x, y, 0.5038), segs=8)
    k.tube(SP, [(-0.085, -0.105, 0.5083), (-0.097, -0.118, 0.5072), (-0.110, -0.134, 0.5072)], 0.0005, fillet=0.006,
           segs=8)
    # ---- clutch actuator on the right cover: shaft, arm, cable, adjuster in a cast lug
    sx_, sy_ = -0.060, -0.142
    k.cyl(BK, 0.010, (sx_, sy_, 0.470), (sx_, sy_, 0.4945), bevel=0.0012, segs=32)
    k.cyl(SP, 0.0048, (sx_, sy_, 0.494), (sx_, sy_, 0.5035), segs=20)
    arm = round_poly([(sx_ - 0.0075, sy_ + 0.0075), (sx_ + 0.0075, sy_ + 0.0075), (sx_ + 0.005, -0.1815),
                      (sx_ - 0.005, -0.1815)], 0.003)
    k.plate(Z, arm, 0.4972, 0.5006, 0.0008, 2)
    k.bolt((sx_, sy_, 0.5006), (0, 0, 1), d=0.005, kind="hex", washer=False, head_h=0.003)
    k.spring(SP, (sx_, sy_, 0.4946), (sx_, sy_, 0.4972), 0.0068, 3, 0.0006)
    k.cyl("brass", 0.0032, (sx_, -0.177, 0.4955), (sx_, -0.177, 0.5025), segs=16)
    k.cyl(SP, 0.0008, (sx_, -0.177, 0.499), (-0.102, -0.177, 0.499), segs=8)
    k.box(BK, (0.012, 0.022, 0.036), (-0.115, -0.141, 0.483), bevel=0.003)
    k.box(BK, (0.012, 0.054, 0.015), (-0.115, -0.158, 0.499), bevel=0.003)
    k.cyl(Z, 0.0035, (-0.1215, -0.177, 0.499), (-0.102, -0.177, 0.499), segs=16)
    k.cyl(Z, 0.0062, (-0.1088, -0.177, 0.499), (-0.1048, -0.177, 0.499), segs=6, bevel=0.0005)
    k.cyl("ano_gold", 0.0050, (-0.1345, -0.177, 0.499), (-0.1212, -0.177, 0.499), segs=24, bevel=0.0008)
    k.cyl(Z, 0.0037, (-0.1410, -0.177, 0.499), (-0.1343, -0.177, 0.499), segs=16)
    k.tube("rubber", [(-0.1405, -0.177, 0.499), (-0.165, -0.177, 0.503), (-0.190, -0.172, 0.530),
                      (-0.202, -0.170, 0.552)], 0.0028, fillet=0.02, segs=10)
    # ---- wiring loom along the top of the gearbox, clips, connectors
    lz = 0.4986
    k.tube("rubber", [(-0.148, -0.100, lz), (-0.148, 0.118, lz), (-0.160, 0.118, 0.509), (-0.171, 0.118, 0.5465),
                      (-0.200, 0.118, 0.5465)], 0.005, fillet=0.012, segs=12)
    k.box("plastic_black", (0.012, 0.016, 0.009), (-0.148, -0.106, lz), bevel=0.0015)
    k.box("plastic_black", (0.016, 0.012, 0.009), (-0.204, 0.118, 0.5465), bevel=0.0015)
    for y in (-0.062, 0.0, 0.074):
        k.box(Z, (0.019, 0.006, 0.0012), (-0.1465, y, lz + 0.0054))
        k.bolt((-0.1392, y, 0.4932), (0, 0, 1), d=0.0035, kind="button", washer=False, mat=Z)
    # ---- ground strap (flat braided copper) from the rear upper lug
    P0, C, P2 = Vector((-0.180, 0.5413)), Vector((-0.214, 0.5413)), Vector((-0.238, 0.578))
    lo, hi = [], []
    for i in range(13):
        t = i / 12
        p = (1 - t) ** 2 * P0 + 2 * t * (1 - t) * C + t ** 2 * P2
        tg = (2 * (1 - t) * (C - P0) + 2 * t * (P2 - C)).normalized()
        nrm = Vector((tg.y, -tg.x))
        if nrm.y < 0:
            nrm = -nrm
        lo.append((p.x, p.y))
        hi.append((p.x + nrm.x * 0.0012, p.y + nrm.y * 0.0012))
    k.plate_xz("copper", lo + hi[::-1], 0.0655, 0.0745, 0.0, 2)
    k.cyl("copper", 0.0062, (-0.188, 0.07, 0.5408), (-0.188, 0.07, 0.5432), segs=20)
    k.bolt((-0.188, 0.07, 0.5430), (0, 0, 1), d=0.006, kind="hex", mat=Z)
    # ---- starter terminal boot
    k.cyl("rubber", 0.0072, (-0.105, -0.020, 0.537), (-0.117, -0.020, 0.5525), segs=20, r2=0.0048, bevel=0.001)


def _detail_plumbing(k):
    Z = "zinc"
    d0 = 0.222
    # fuel feed: AN fitting on the rail end + braided hose stub
    k.cyl("ano_red", 0.0062, Lp(-0.127, 0.087, d0 + 0.05), Lp(-0.127, 0.096, d0 + 0.05), segs=6, bevel=0.0006)
    k.cyl(AM, 0.0050, Lp(-0.127, 0.096, d0 + 0.05), Lp(-0.127, 0.104, d0 + 0.05), segs=20)
    E = Vector((-0.172, 0.127, 0.577))
    k.tube("braid", [Lp(-0.127, 0.102, d0 + 0.05), Lp(-0.127, 0.127, d0 + 0.05), Vector((-0.100, 0.127, 0.585)), E],
           0.0042, fillet=0.02, segs=16, res=10)
    dv = (E - Vector((-0.100, 0.127, 0.585))).normalized()
    k.cyl(AM, 0.0052, E - dv * 0.012, E - dv * 0.003, segs=20)
    k.cyl("ano_blue", 0.0062, E - dv * 0.004, E + dv * 0.005, segs=6, bevel=0.0006)
    # injector feed caps on the rail
    for s in (1, -1):
        k.cyl(Z, 0.0058, Lp(-0.127, s * 0.075 - 0.006, d0 + 0.05), Lp(-0.127, s * 0.075 + 0.006, d0 + 0.05), segs=20,
              bevel=0.0006)
    # breather: T-piece on the hose and a small filter
    by, bz = -0.1245, 0.5455
    k.cyl(Z, 0.0072, (-0.130, by, bz - 0.0075), (-0.130, by, bz + 0.0075), segs=16, bevel=0.0008)
    k.cyl(Z, 0.0045, (-0.130, by, bz), (-0.146, by, bz), segs=16)
    k.cyl(SD, 0.0100, (-0.146, by, bz), (-0.168, by, bz), segs=32)
    for i in range(8):
        k.torus(SD, 0.0101, 0.0009, (-0.1482 - 0.0025 * i, by, bz), (1, 0, 0), segs=32, rsegs=6)
    k.cyl(AM, 0.0115, (-0.1440, by, bz), (-0.1472, by, bz), segs=32, bevel=0.0008)
    k.cyl(AM, 0.0115, (-0.1668, by, bz), (-0.1700, by, bz), segs=32, bevel=0.0008)
    # spark plug lead collars (plain marker bands, one for cylinder 1, two for cylinder 2)
    px, pz = 0.066 + math.sin(math.radians(25.0)) * 0.035, 0.2775 + math.cos(math.radians(25.0)) * 0.035
    for s in (1, -1):
        k.cyl("white_paint", 0.0046, Lp(px, s * 0.1095, pz), Lp(px, s * 0.1175, pz), segs=16)
        for j in range(1 if s < 0 else 2):
            y = s * (0.1118 + 0.0026 * j)
            k.cyl("ano_red", 0.0048, Lp(px, y, pz), Lp(px, y + s * 0.0010, pz), segs=16)


def _detail_intake(k):
    Z = "zinc"
    d0 = 0.222
    back = Ld(-1, 0, 0)
    # return springs on the throttle shaft, sync rod with rose joints
    for s in (1, -1):
        k.spring(SP, Lp(-0.127, s * 0.008, d0), Lp(-0.127, s * 0.040, d0), 0.0058, 8, 0.0006)
        k.cyl(Z, 0.0022, Lp(-0.127, s * 0.041, d0), Lp(-0.147, s * 0.041, d0 - 0.012), segs=10)
        k.cyl(Z, 0.0050, Lp(-0.127, s * 0.0395, d0), Lp(-0.127, s * 0.0425, d0), segs=16)
        k.cyl(Z, 0.0040, Lp(-0.147, s * 0.0380, d0 - 0.012), Lp(-0.147, s * 0.0440, d0 - 0.012), segs=16, bevel=0.0012)
        k.cyl("ano_gold", 0.0030, Lp(-0.147, s * 0.030, d0 - 0.012), Lp(-0.147, s * 0.0345, d0 - 0.012), segs=6)
    k.cyl(SP, 0.0018, Lp(-0.147, -0.0385, d0 - 0.012), Lp(-0.147, 0.0385, d0 - 0.012), segs=10)
    # idle screw with spring (right body, underside)
    k.cyl("brass", 0.0040, Lp(-0.127, -0.075, d0 - 0.0285), Lp(-0.127, -0.075, d0 - 0.041), segs=16, bevel=0.0006)
    k.spring(SP, Lp(-0.127, -0.075, d0 - 0.0288), Lp(-0.127, -0.075, d0 - 0.0365), 0.0050, 4, 0.0005)
    # throttle position sensor (left end of the shaft) with wire stub
    k.box("plastic_black", (0.024, 0.010, 0.022), (-0.127, 0.114, d0), bevel=0.002, m=M_CYL)
    k.box("plastic_black", (0.010, 0.008, 0.009), (-0.127, 0.114, d0 - 0.0145), bevel=0.0012, m=M_CYL)
    for du_, dd_ in ((0.008, 0.007), (-0.008, -0.007)):
        k.bolt(Lp(-0.127 + du_, 0.119, d0 + dd_), (0, 1, 0), d=0.003, kind="button", washer=False, mat=Z)
    k.tube("rubber", [Lp(-0.127, 0.114, d0 - 0.018), Lp(-0.127, 0.114, d0 - 0.036), Vector((-0.062, 0.118, 0.546))],
           0.002, fillet=0.008, segs=8)
    # velocity stack mesh screens
    um, Rm = -0.1945, 0.0362
    for s in (1, -1):
        yc_ = s * 0.075
        for i in range(-9, 10):
            o = i * 0.0038
            Lw = 2 * math.sqrt(Rm * Rm - o * o)
            k.box(SP, (0.0005, Lw, 0.0005), (um, yc_, d0 + o), m=M_CYL)
            k.box(SP, (0.0005, 0.0005, Lw), (um + 0.0005, yc_ + o, d0), m=M_CYL)
        k.torus(SP, Rm, 0.0009, Lp(um, yc_, d0), back, segs=48, rsegs=6)


def _detail_exhaust(k):
    Z = "zinc"
    yc, zc, R = -0.255, 0.205, 0.048
    z_up, z_lo = 0.224, 0.186
    # weld beads + slip-joint springs where the headers enter the collector
    yo = yc - 0.0212 - 0.0030
    for z in (z_up, z_lo):
        k.torus("steel_brushed", 0.0190, 0.0012, (0.1612, yc, z), (1, 0, 0), segs=40)
        k.spring("steel_brushed", (0.172, yo, z), (0.146, yo, z), 0.0022, 10, 0.00055)
        for x in (0.1735, 0.1445):
            k.cyl("steel_brushed", 0.0012, (x, yc - 0.017, z), (x, yo - 0.0012, z), segs=8)
    # O2 sensor on the collector with wire stub to a connector on the case
    ox, zt = 0.104, 0.2372
    k.cyl("steel_brushed", 0.0085, (ox, yc, zt - 0.005), (ox, yc, zt + 0.004), segs=24, bevel=0.0008)
    k.cyl(Z, 0.0075, (ox, yc, zt + 0.004), (ox, yc, zt + 0.011), segs=6, bevel=0.0006)
    k.cyl("steel_brushed", 0.0055, (ox, yc, zt + 0.011), (ox, yc, zt + 0.030), segs=20, bevel=0.0008)
    k.cyl("rubber", 0.0048, (ox, yc, zt + 0.029), (ox, yc, zt + 0.044), segs=16, r2=0.003, bevel=0.0008)
    k.tube("rubber", [(ox, yc, zt + 0.043), (ox, yc, 0.305), (0.110, -0.150, 0.366), (0.116, -0.139, 0.384)], 0.002,
           fillet=0.02, segs=8)
    k.box("plastic_black", (0.008, 0.006, 0.014), (0.116, -0.139, 0.390), bevel=0.0015)
    # clamp band at the muffler inlet
    k.add(lathe_x(k.nm("clamp2"), [(0.0305, 0.0296), (0.0305, 0.0306), (0.0298, 0.0312), (0.0202, 0.0312),
                                   (0.0195, 0.0306), (0.0195, 0.0296)], yc, zc, k.col, segs=64), Z)
    k.box(Z, (0.009, 0.012, 0.008), (0.025, yc - 0.004, zc + 0.034), bevel=0.0015)
    k.bolt((0.025, yc + 0.002, zc + 0.034), (0, 1, 0), d=0.005, kind="hex", washer=False, mat=SD)
    # subtle heat bands
    k.add(lathe_x(k.nm("blue1"), [(0.078, 0.02735), (0.060, 0.02735)], yc, zc, k.col, segs=64), "ti_blued")
    k.add(lathe_x(k.nm("blue2"), [(-0.2380, R + 0.0003), (-0.2615, R + 0.0003)], yc, zc, k.col, segs=72), "ti_blued")
    # extra rivets on the tail band
    for i in range(10):
        a = 2 * math.pi * i / 10
        n = Vector((0, math.cos(a), math.sin(a)))
        if n.y > 0.75:
            continue
        k.bolt(Vector((-0.2692, yc, zc)) + n * 0.0497, n, d=0.0035, kind="button", washer=False, mat=Z)
    # slotted carbon heat shield on stand-offs, 6 mm off the can
    rs = 0.0540
    x0, x1 = -0.232, -0.058
    a0, a1 = math.radians(102.0), math.radians(216.0)
    nx, na = 70, 30
    bm = bmesh.new()
    V = [[bm.verts.new((x0 + (x1 - x0) * i / nx, yc + rs * math.cos(a0 + (a1 - a0) * j / na),
                        zc + rs * math.sin(a0 + (a1 - a0) * j / na))) for j in range(na + 1)] for i in range(nx + 1)]
    for i in range(nx):
        for j in range(na):
            row = (j - 2) // 5
            ii = (i + (5 if row % 2 else 0)) % 10
            if (j - 2) % 5 == 2 and 2 <= ii <= 7 and 4 <= i < nx - 4 and 3 <= j < na - 3:
                continue
            bm.faces.new((V[i][j], V[i + 1][j], V[i + 1][j + 1], V[i][j + 1]))
    sh = geo.obj_from_bm(k.nm("shield"), bm, k.col, smooth=None)
    m = sh.modifiers.new("Solid", "SOLIDIFY")
    m.thickness = 0.0012
    m.offset = 0.0
    geo.apply_mods(sh)
    geo.set_smooth(sh, 40.0)
    k.add(sh, "carbon")
    for x in (-0.214, -0.076):
        for a in (math.radians(124.0), math.radians(198.0)):
            n = Vector((0, math.cos(a), math.sin(a)))
            c = Vector((x, yc, zc))
            k.cyl(SP, 0.0032, c + n * (R - 0.0005), c + n * (rs - 0.0004), segs=12)
            k.bolt(c + n * (rs + 0.0006), n, d=0.0035, kind="button", washer=True, mat=SP)


def _detail_more(k):
    Z = "zinc"
    ax, az = CRANK.x, CRANK.z
    bx, bz, yr = -0.065, 0.380, -0.149
    # ---- clutch dome: radial cast ribs on the flank, ring of small screws around the cap
    base = orient_m((bx, yr, bz), (0, -1, 0))
    sl = math.atan2(0.015, 0.023)
    for i in range(8):
        m = base @ Matrix.Rotation(2 * math.pi * (i + 0.5) / 8, 4, "Z") @ Matrix.Translation((0.0635, 0, 0.0290)) \
            @ Matrix.Rotation(sl, 4, "Y")
        k.box(BK, (0.027, 0.0046, 0.0044), (0, 0, 0), bevel=0.0013, m=m)
    k.lathe(BK, [(0.0335, 0.0), (0.0335, 0.0022, 0.0008), (0.0445, 0.0022, 0.0008), (0.0445, 0.0)],
            (bx, yr - 0.0358, bz), (0, -1, 0), segs=72)
    k.bolt_circle((bx, yr - 0.038, bz), (0, -1, 0), 0.039, 6, phase=math.radians(15), d=0.004, mat=Z)
    # ignition cover: centre inspection plug + machined grooves
    k.cyl(Z, 0.0080, (ax, yr - 0.0153, az), (ax, yr - 0.0186, az), segs=6, bevel=0.0006)
    for rr in (0.026, 0.031):
        k.torus(SD, rr, 0.0005, (ax, yr - 0.0141, az), (0, 1, 0), segs=64, rsegs=6)
    # ---- crankcase half bolts on bosses, front and rear
    for y in (-0.122, -0.021, 0.021, 0.122):
        k.cyl(BK, 0.0068, (0.1500, y, 0.318), (0.1500, y, 0.3655), segs=20, bevel=0.001)
        k.bolt((0.1500, y, 0.3655), (0, 0, 1), d=0.0055, kind="hex", mat=Z, head_h=0.004)
    for y in (-0.120, 0.045):
        k.cyl(BK, 0.0062, (-0.2395, y, 0.383), (-0.2395, y, 0.4225), segs=20, bevel=0.001)
        k.bolt((-0.2395, y, 0.4225), (0, 0, 1), d=0.005, kind="hex", mat=Z, head_h=0.0035)
    # ---- cylinder: hold-down nuts, fin webs, rubber fin dampers, casting dots on the head
    for su in (1, -1):
        for sy in (1, -1):
            k.bolt(Lp(su * 0.070, sy * 0.142, 0.113), Ld(0, 0, 1), d=0.006, kind="hex", mat=Z, head_h=0.0038)
            for d0, d1 in ((0.1195, 0.1792), (0.1975, 0.2428)):
                k.box(BK, (0.0026, 0.026, d1 - d0), (su * 0.030, sy * 0.147, (d0 + d1) / 2), m=M_CYL)
                k.box(BK, (0.020, 0.0026, d1 - d0), (su * 0.066, sy * 0.150, (d0 + d1) / 2), m=M_CYL)
                k.cyl("rubber", 0.0034, Lp(su * 0.052, sy * 0.1585, d0 + 0.003), Lp(su * 0.052, sy * 0.1585, d1 - 0.003),
                      segs=12)
    for sy in (1, -1):
        for i in range(4):
            k.cyl(BK, 0.0014, Lp(0.040 + 0.006 * i, sy * 0.1555, 0.2565), Lp(0.040 + 0.006 * i, sy * 0.1574, 0.2565),
                  segs=12)
        k.box(BK, (0.012, 0.0026, 0.0022), (0.049, sy * 0.1563, 0.2700), bevel=0.0006, m=M_CYL)
    k.box(AM, (0.0012, 0.030, 0.011), (0.0752, 0, 0.2635), bevel=0.0004, m=M_CYL)
    # ---- left: stator cable with grommet, detent plug, bevel box fill / level plugs, starter plate
    k.box("rubber", (0.013, 0.010, 0.012), (-0.0067, 0.1425, 0.4067), bevel=0.003)
    k.tube("rubber", [(-0.0067, 0.146, 0.4067), (-0.016, 0.1395, 0.425), (-0.030, 0.1395, 0.466),
                      (-0.040, 0.128, 0.4945), (-0.040, 0.112, 0.4975)], 0.0035, fillet=0.012, segs=10)
    k.box("plastic_black", (0.012, 0.014, 0.008), (-0.040, 0.108, 0.4972), bevel=0.0015)
    k.box(Z, (0.007, 0.0014, 0.009), (-0.023, 0.1436, 0.4455))
    k.bolt((-0.0165, 0.135, 0.4455), (0, 1, 0), d=0.0035, kind="button", washer=False, mat=Z)
    k.cyl("copper", 0.0095, (-0.086, 0.151, 0.442), (-0.086, 0.1520, 0.442), segs=24)
    k.cyl(Z, 0.0078, (-0.086, 0.1518, 0.442), (-0.086, 0.1565, 0.442), segs=6, bevel=0.0006)
    k.cyl(BK, 0.011, (-0.170, 0.1315, 0.455), (-0.170, 0.1315, 0.4595), bevel=0.001)
    k.cyl(AM, 0.0080, (-0.170, 0.1315, 0.459), (-0.170, 0.1315, 0.4645), segs=6, bevel=0.0006)
    k.cyl("copper", 0.0085, (-0.150, 0.1775, 0.372), (-0.150, 0.1786, 0.372), segs=24)
    k.cyl(Z, 0.0068, (-0.150, 0.1784, 0.372), (-0.150, 0.1828, 0.372), segs=6, bevel=0.0006)
    k.box("alu_brushed", (0.016, 0.034, 0.0010), (-0.085, 0.020, 0.5478))
    for y in (0.006, 0.034):
        k.cyl(SP, 0.0012, (-0.085, y, 0.5478), (-0.085, y, 0.5490), segs=10)
    # alternator cover: radial cast bars on the face; drain plug safety wire; coil connector
    basel = orient_m((ax, 0.135 + 0.046, az), (0, 1, 0))
    for i in range(6):
        m = basel @ Matrix.Rotation(2 * math.pi * (i + 0.5) / 6, 4, "Z") @ Matrix.Translation((0.0415, 0, 0.0004))
        k.box(BK, (0.017, 0.0044, 0.0036), (0, 0, 0), bevel=0.0012, m=m)
    k.tube(SP, [(-0.090, 0.1133, 0.222), (-0.100, 0.1150, 0.244), (-0.115, 0.1100, 0.2640)], 0.0005, fillet=0.008,
           segs=8)
    k.box("plastic_black", (0.008, 0.016, 0.009), (-0.1065, 0.0, 0.262), bevel=0.0015, m=M_CYL)
    k.tube("rubber", [Lp(-0.110, 0.0, 0.262), Lp(-0.118, 0.0, 0.262), Lp(-0.118, 0.0, 0.180)], 0.0022, fillet=0.006,
           segs=8)
    # ---- worm-drive clamps on the breather hose
    k.cyl(Z, 0.0064, Lp(-0.0585, -0.105, 0.284), Lp(-0.0635, -0.105, 0.284), segs=20)
    k.box(Z, (0.0045, 0.0075, 0.0045), (-0.061, -0.105, 0.2915), bevel=0.0008, m=M_CYL)
    k.box(Z, (0.0075, 0.0045, 0.0045), (-0.130, -0.1105, 0.5092), bevel=0.0008)


# ----------------------------------------------------------------------------- entry point
def build(col):
    k = _Kit(col)
    root = geo.empty("engine_root", (0, 0, 0), col)
    _crankcase(k)
    _covers(k)
    _cylinders(k)
    _intake(k)
    _ignition(k)
    _ancillaries(k)
    _exhaust(k)
    _detail_cases(k)
    _detail_plumbing(k)
    _detail_intake(k)
    _detail_exhaust(k)
    _detail_more(k)
    k.finish(root)
    return {"engine": root}
