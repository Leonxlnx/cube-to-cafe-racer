"""Wheels (tyres, wire-spoked rims, hubs, floating brake discs) and the rear bevel-drive housing.

build(col) -> {"front_wheel", "rear_wheel", "rear_drive"}  (root Empties, identity rotation)
  front_wheel / rear_wheel : at common.FW_C / common.RW_C, everything that ROTATES about the local Y axle.
  rear_drive               : at common.RW_C, non-rotating final-drive housing + rear caliper.

Wheel meshes are modelled in a "wheel frame" whose Z axis is the axle; the objects are rotated
(-90 deg about X) so their local Z points along world +Y: the *_turned materials then brush
concentrically around the axle.  wheel frame (x, y, z)  ->  world (x, z, -y)  (relative to the centre).
"""
import math
import bpy
import bmesh
from mathutils import Vector, Matrix

import common
import geo
import mats

TAU = 2.0 * math.pi
ROT_AXLE = (-math.pi / 2.0, 0.0, 0.0)          # object local Z -> world +Y
M_AXLE = Matrix.Rotation(-math.pi / 2.0, 4, "X")  # same, as a matrix (wheel frame -> world orientation)


def smoothstep(a, b, x):
    if b == a:
        return 1.0 if x >= b else 0.0
    t = min(1.0, max(0.0, (x - a) / (b - a)))
    return t * t * (3.0 - 2.0 * t)


def lerp(a, b, t):
    return a + (b - a) * t


# ============================================================== mesh accumulator
class MB:
    """One output object: a bmesh plus an ordered list of material names."""

    def __init__(self, name, mat_names):
        self.name = name
        self.bm = bmesh.new()
        self.mats = list(mat_names)

    def mi(self, mat):
        return self.mats.index(mat)

    def add(self, bm2, mat=None, matrix=None):
        """Append bmesh bm2 (it is freed).  mat: material name for all its faces (None keeps indices)."""
        if matrix is not None:
            bm2.transform(matrix)
        me = bpy.data.meshes.new("_tmp")
        bm2.to_mesh(me)
        bm2.free()
        n0 = len(self.bm.faces)
        self.bm.from_mesh(me)
        bpy.data.meshes.remove(me)
        if mat is not None:
            self.bm.faces.ensure_lookup_table()
            idx = self.mi(mat)
            for i in range(n0, len(self.bm.faces)):
                self.bm.faces[i].material_index = idx

    def finish(self, col, loc=(0, 0, 0), rot=(0, 0, 0), smooth=50.0):
        bm = self.bm
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
        thr = math.radians(smooth)
        for f in bm.faces:
            f.smooth = True
        for e in bm.edges:
            if len(e.link_faces) != 2:
                e.smooth = False
                continue
            try:
                if e.calc_face_angle() > thr:
                    e.smooth = False
            except ValueError:
                pass
        me = bpy.data.meshes.new(self.name)
        bm.to_mesh(me)
        bm.free()
        ob = bpy.data.objects.new(self.name, me)
        col.objects.link(ob)
        for m in self.mats:
            me.materials.append(mats.get(m))
        ob.location = loc
        ob.rotation_euler = rot
        return ob


# ============================================================== generators (write into a bmesh)
def lathe_bm(bm, prof, segs, mi, mat4=None, th0=0.0, th1=None, caps=True, closed=False):
    """Revolve a profile [(r, z) or (r, z, mi_segment)] around Z.  Write profiles counter-clockwise
    in the (r, z) plane (solid on the left).  Full turn when th1 is None (th = th0 + ...)."""
    full = th1 is None
    n_th = segs if full else segs + 1
    span = TAU if full else (th1 - th0)
    rings = []
    for p in prof:
        r, z = p[0], p[1]
        if r <= 1e-9:
            rings.append([bm.verts.new((0.0, 0.0, z))] * n_th)
        else:
            ring = []
            for i in range(n_th):
                a = th0 + span * i / segs
                ring.append(bm.verts.new((r * math.cos(a), r * math.sin(a), z)))
            rings.append(ring)
    npf = len(prof)
    pairs = [(k, k + 1) for k in range(npf - 1)] + ([(npf - 1, 0)] if closed else [])
    for (k, k1) in pairs:
        ra, rb = rings[k], rings[k1]
        m = prof[k][2] if len(prof[k]) > 2 else mi
        for i in range(segs):
            j = (i + 1) % n_th if full else i + 1
            vs = [ra[i], ra[j], rb[j], rb[i]]
            uniq = []
            for v in vs:
                if v not in uniq:
                    uniq.append(v)
            if len(uniq) < 3:
                continue
            try:
                f = bm.faces.new(uniq)
                f.material_index = m
            except ValueError:
                pass
    if not full and caps:
        for idx in (0, n_th - 1):
            vs = []
            for ring in rings:
                if ring[idx] not in vs:
                    vs.append(ring[idx])
            if len(vs) >= 3:
                try:
                    f = bm.faces.new(vs)
                    f.material_index = mi
                except ValueError:
                    pass
    if mat4 is not None:
        vs = [v for ring in rings for v in ring]
        seen = set()
        for v in vs:
            if id(v) in seen:
                continue
            seen.add(id(v))
            v.co = mat4 @ v.co
    return rings


def sweep_bm(bm, pts, rad, sides, mi, caps=True, rads=None):
    """Round tube along a polyline (parallel-transport frames)."""
    P = [Vector(p) for p in pts]
    n = len(P)
    T = []
    for i in range(n):
        if i == 0:
            t = P[1] - P[0]
        elif i == n - 1:
            t = P[-1] - P[-2]
        else:
            t = (P[i + 1] - P[i]).normalized() + (P[i] - P[i - 1]).normalized()
        T.append(t.normalized())
    a = Vector((0, 0, 1)) if abs(T[0].z) < 0.9 else Vector((1, 0, 0))
    N = T[0].cross(a).normalized()
    rings = []
    for i in range(n):
        if i > 0:
            ax = T[i - 1].cross(T[i])
            if ax.length > 1e-9:
                N = Matrix.Rotation(T[i - 1].angle(T[i]), 3, ax.normalized()) @ N
        B = T[i].cross(N)
        rr = rad * (rads[i] if rads else 1.0)
        rings.append([bm.verts.new(P[i] + (N * math.cos(TAU * k / sides) + B * math.sin(TAU * k / sides)) * rr)
                      for k in range(sides)])
    for i in range(n - 1):
        for k in range(sides):
            k2 = (k + 1) % sides
            f = bm.faces.new((rings[i][k], rings[i][k2], rings[i + 1][k2], rings[i + 1][k]))
            f.material_index = mi
    if caps:
        for ring in (rings[0], rings[-1]):
            f = bm.faces.new(ring)
            f.material_index = mi
    return rings


def fillet_poly(pts, rad, res=6):
    """Round the inner corners of a 3D polyline (quadratic Bezier arcs)."""
    P = [Vector(p) for p in pts]
    if len(P) < 3:
        return P
    Q = [P[0]]
    for i in range(1, len(P) - 1):
        a, b, c = P[i - 1], P[i], P[i + 1]
        d1, d2 = a - b, c - b
        l1, l2 = d1.length, d2.length
        d1.normalize()
        d2.normalize()
        ang = d1.angle(d2)
        if ang > math.pi - 1e-3:
            Q.append(b)
            continue
        t = min(rad / math.tan(ang / 2.0), l1 * 0.48, l2 * 0.48)
        p1, p2 = b + d1 * t, b + d2 * t
        for k in range(res + 1):
            u = k / res
            Q.append((1 - u) ** 2 * p1 + 2 * u * (1 - u) * b + u ** 2 * p2)
    Q.append(P[-1])
    return Q


def plate_bm(loops, thick, bevel, bres=2):
    """Extruded 2D shape (list of closed (x, y) loops, holes allowed) with rounded edges,
    centred on z = 0, thickness `thick` along Z.  Cap-boundary edges are marked sharp."""
    cu = bpy.data.curves.new("_plate", "CURVE")
    cu.dimensions = "2D"
    cu.fill_mode = "BOTH"
    cu.extrude = max(thick / 2.0 - bevel, 0.0)
    cu.bevel_depth = bevel
    cu.bevel_resolution = bres
    for loop in loops:
        sp = cu.splines.new("POLY")
        sp.points.add(len(loop) - 1)
        for i, (x, y) in enumerate(loop):
            sp.points[i].co = (x, y, 0.0, 1.0)
        sp.use_cyclic_u = True
    ob = bpy.data.objects.new("_plate", cu)
    bpy.context.scene.collection.objects.link(ob)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    bpy.data.objects.remove(ob, do_unlink=True)
    bpy.data.curves.remove(cu)
    bm = bmesh.new()
    bm.from_mesh(me)
    bpy.data.meshes.remove(me)
    bm.normal_update()
    for e in bm.edges:
        lf = e.link_faces
        if len(lf) == 2:
            c = [abs(f.normal.z) > 0.9995 for f in lf]
            if c[0] != c[1]:
                e.smooth = False
    return bm


def circle2d(cx, cy, r, n):
    return [(cx + r * math.cos(TAU * i / n), cy + r * math.sin(TAU * i / n)) for i in range(n)]


def sq_sector(r0, r1, th_c, half_fn, npts=48, ex=4.0):
    """Rounded annular-sector outline: superellipse in polar (r, theta) space."""
    pts = []
    rc, dr = (r0 + r1) / 2.0, (r1 - r0) / 2.0
    for i in range(npts):
        a = TAU * i / npts
        ca, sa = math.cos(a), math.sin(a)
        r = rc + dr * math.copysign(abs(sa) ** (2.0 / ex), sa)
        th = th_c + half_fn(r) * math.copysign(abs(ca) ** (2.0 / ex), ca)
        pts.append((r * math.cos(th), r * math.sin(th)))
    return pts


def frame_matrix(origin, ex, ey, ez):
    """4x4 matrix whose columns are the given axes (maps local -> parent)."""
    m = Matrix.Identity(4)
    for i in range(3):
        m[i][0], m[i][1], m[i][2], m[i][3] = ex[i], ey[i], ez[i], origin[i]
    return m


def along(p0, d, up_hint=(0, 0, 1)):
    """Matrix placing a local-Z-axis part at p0 pointing along d."""
    d = Vector(d).normalized()
    h = Vector(up_hint)
    if abs(d.dot(h)) > 0.95:
        h = Vector((1, 0, 0)) if abs(d.x) < 0.9 else Vector((0, 1, 0))
    x = h.cross(d).normalized()
    y = d.cross(x)
    return frame_matrix(Vector(p0), x, y, d)


def bolt_bm(bm, mi, d=0.008, head_h=None, mat4=None, segs=20):
    """Socket-head cap screw head (+ thin washer) standing on z = 0, pointing +Z."""
    hh = head_h or d * 0.85
    rr = d * 0.80
    rw = d * 1.05
    tw = d * 0.12
    k = 0.0006
    prof = [(0.0, 0.0), (rw, 0.0), (rw, tw), (rr, tw), (rr, tw + hh - k), (rr - k, tw + hh),
            (rr * 0.50, tw + hh), (rr * 0.42, tw + hh - 0.0004), (rr * 0.42, tw + hh * 0.45), (0.0, tw + hh * 0.45)]
    lathe_bm(bm, prof, segs, mi, mat4=mat4)


def hexnut_prof_bm(bm, mi, r_af, h, mat4=None, chamfer=0.0008, bevel=0.0005):
    """Hex prism (across-flats radius r_af) centred on z in [0, h], rounded edges."""
    rc = r_af / math.cos(math.pi / 6)
    loop = [(rc * math.cos(TAU * i / 6 + math.pi / 6), rc * math.sin(TAU * i / 6 + math.pi / 6)) for i in range(6)]
    b2 = plate_bm([loop], h, bevel, 1)
    bmesh.ops.translate(b2, vec=(0, 0, h / 2), verts=b2.verts)
    if mat4 is not None:
        b2.transform(mat4)
    me = bpy.data.meshes.new("_t")
    b2.to_mesh(me)
    b2.free()
    n0 = len(bm.faces)
    bm.from_mesh(me)
    bpy.data.meshes.remove(me)
    bm.faces.ensure_lookup_table()
    for i in range(n0, len(bm.faces)):
        bm.faces[i].material_index = mi


# ============================================================== tyre
def _tyre_half_profile(R, hw, bw, rb):
    """Dense half cross-section (y, r) from the crown (y = 0) to the left bead, arc-length table."""
    rm = rb + 0.48 * (R - rb)
    bv = R - rm
    pts = []
    for i in range(160):
        ph = math.pi / 2 * (1 - i / 159.0)
        pts.append((hw * math.cos(ph), rm + bv * math.sin(ph)))
    i_mw = len(pts) - 1
    rt = rb + 0.015
    B0, B1 = Vector((hw, rm)), Vector((hw, rm - 0.5 * (rm - rt)))
    B2, B3 = Vector((bw + 0.004, rt + 0.35 * (rm - rt))), Vector((bw + 0.0005, rt))
    for i in range(1, 61):
        t = i / 60.0
        p = (1 - t) ** 3 * B0 + 3 * (1 - t) ** 2 * t * B1 + 3 * (1 - t) * t * t * B2 + t ** 3 * B3
        pts.append((p.x, p.y))
    for i in range(1, 9):
        t = i / 8.0
        pts.append((bw + 0.0005 * (1 - t), rt - t * (rt - rb - 0.003)))
    s = [0.0]
    for i in range(1, len(pts)):
        s.append(s[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
    return pts, s, s[i_mw], i_mw


def _prof_eval(pts, s, x):
    import bisect
    i = bisect.bisect_right(s, x) - 1
    i = max(0, min(i, len(s) - 2))
    t = (x - s[i]) / max(s[i + 1] - s[i], 1e-12)
    y = lerp(pts[i][0], pts[i + 1][0], t)
    r = lerp(pts[i][1], pts[i + 1][1], t)
    i0, i1 = max(0, i - 1), min(len(pts) - 1, i + 2)
    ty, tr = pts[i1][0] - pts[i0][0], pts[i1][1] - pts[i0][1]
    L = math.hypot(ty, tr)
    return y, r, -tr / L, ty / L          # position + outward normal (ny, nr)


def tyre_bm(R, W, bw, rb, n_per=18, wmax=0.0055, dmax=0.0035, a_sh=2.4, b_sh=0.7):
    """Motorcycle tyre in the wheel frame (axle = Z).  Tread = sheared quad grid whose columns follow
    the swept groove family, so the grooves are real, crisp, tapering modelled channels."""
    bm = bmesh.new()
    hw = W / 2.0
    pts, s_tab, s_mw, i_mw = _tyre_half_profile(R, hw, bw, rb)
    s_end = s_tab[-1]
    Ut = 0.86 * s_mw
    rm = rb + 0.48 * (R - rb)

    def s_at_r(rt):
        for i in range(i_mw, len(pts) - 1):
            if pts[i][1] >= rt >= pts[i + 1][1]:
                t = (pts[i][1] - rt) / max(pts[i][1] - pts[i + 1][1], 1e-12)
                return lerp(s_tab[i], s_tab[i + 1], t)
        return s_end

    # ---------------- rows (u > 0 = left = +Z)
    half = []
    nt = max(8, int(round(Ut / 0.0024)))
    for i in range(1, nt + 1):
        half.append((Ut * i / nt, 0.0))
    ribs = [s_at_r(rm - 0.012), s_at_r(rb + 0.029)]
    sw = []
    ns = max(6, int(round((s_end - Ut) / 0.0034)))
    for i in range(1, ns + 1):
        x = Ut + (s_end - Ut) * i / ns
        if any(abs(x - rc) < 0.0021 for rc in ribs):
            continue
        sw.append((x, 0.0))
    for rc in ribs:
        for dx, off in ((-0.0014, 0.0), (-0.0006, 0.00055), (0.0006, 0.00055), (0.0014, 0.0)):
            sw.append((rc + dx, off))
    half += sorted(sw)
    rows = [(u, o) for (u, o) in reversed(half)] + [(0.0, 0.0)] + [(-u, o) for (u, o) in half]

    # ---------------- columns
    P = TAU * R / n_per
    NCP = 24
    groups = [(0.0, 1, (0.06, 0.82)), (P / 4, -1, (0.06, 0.82)), (P / 2, 1, (0.38, 1.0)), (3 * P / 4, -1, (0.38, 1.0))]
    hmin = 0.0015

    def g(s):
        s = min(s, Ut)
        return a_sh * s - (a_sh - b_sh) * s * s / (2 * Ut)

    def layout(u):
        s = abs(u)
        side = 1 if u >= 0 else -1
        lam = 1.0 - smoothstep(Ut, Ut + 0.012, s)
        gp = a_sh - (a_sh - b_sh) * min(s, Ut) / Ut
        k = math.sqrt(1.0 + gp * gp)
        grp = []
        for (c, gside, (ua, ub)) in groups:
            h, e, D = hmin, hmin * 0.5, 0.0
            if gside == side and ua * Ut < s < ub * Ut and lam > 0:
                tau = (s - ua * Ut) / ((ub - ua) * Ut)
                sh = math.sin(math.pi * tau)
                hp = wmax / 2 * sh ** 0.55
                h = max(hp * k, hmin)
                e = min(0.00065 * k, 0.45 * h)
                D = dmax * sh ** 0.35 * lam
            offs = [-h, -h + e, 0.0, h - e, h]
            deps = [0.0, 0.88 * D, D, 0.88 * D, 0.0]
            grp.append((c, offs, deps))
        cols = []
        shift = -g(s)
        for p in range(n_per):
            base = p * P
            for gi, (c, offs, deps) in enumerate(grp):
                for kk in range(5):
                    uni = (6 * gi + kk - 2) * P / NCP
                    tt = lerp(uni, c + offs[kk], lam)
                    cols.append((base + tt + shift, deps[kk]))
                cols.append((base + c + P / 8 + shift, 0.0))
        return cols

    # ---------------- vertices
    grid = []
    for (u, off) in rows:
        y, r, ny, nr = _prof_eval(pts, s_tab, abs(u))
        if u < 0:
            y, ny = -y, -ny
        row = []
        for (t, dep) in layout(u):
            th = t / R
            d = off - dep
            rr, zz = r + nr * d, y + ny * d
            row.append(bm.verts.new((rr * math.cos(th), rr * math.sin(th), zz)))
        grid.append(row)
    # closing rows under the beads (hidden inside the rim)
    for (y, r) in ((-(bw - 0.004), rb + 0.0004), (bw - 0.004, rb + 0.0004)):
        row = []
        for (t, dep) in layout(10.0):
            th = t / R
            row.append(bm.verts.new((r * math.cos(th), r * math.sin(th), y)))
        grid.append(row)
    nr_ = len(grid)
    nc = len(grid[0])
    for i in range(nr_):
        a, b = grid[i], grid[(i + 1) % nr_]
        for j in range(nc):
            j2 = (j + 1) % nc
            bm.faces.new((a[j], a[j2], b[j2], b[j]))

    # ---------------- wear-indicator pads (small raised triangles on the shoulder), both sides
    for side in (1, -1):
        sx = Ut + 0.0055
        y, r, ny, nr = _prof_eval(pts, s_tab, sx)
        y2, r2, _, _ = _prof_eval(pts, s_tab, sx - 0.002)
        tdir = Vector((r2 - r, 0, (y2 - y)))
        for k in range(6):
            th = TAU * (k + 0.25 + (0.5 if side < 0 else 0.0)) / 6
            ct, st_ = math.cos(th), math.sin(th)
            n3 = Vector((nr * ct, nr * st_, ny * side))
            t3 = Vector((tdir.x * ct, tdir.x * st_, tdir.z * side)).normalized()
            p3 = Vector((r * ct, r * st_, y * side)) + n3 * 0.0001
            b3 = n3.cross(t3)
            tri = [(0.0042, 0.0), (-0.0026, 0.0034), (-0.0026, -0.0034)]
            pb = plate_bm([tri], 0.0018, 0.0004, 1)
            pb.transform(frame_matrix(p3, t3, b3, n3))
            me = bpy.data.meshes.new("_t")
            pb.to_mesh(me)
            pb.free()
            bm.from_mesh(me)
            bpy.data.meshes.remove(me)
    return bm


# ============================================================== rim, nipples, valve, spokes
R_UNDER = 0.0225          # rim underside (nipple seat) depth below the bead seat


def _ccw(prof):
    a = 0.0
    for i in range(len(prof)):
        r0, z0 = prof[i][0], prof[i][1]
        r1, z1 = prof[(i + 1) % len(prof)][0], prof[(i + 1) % len(prof)][1]
        a += r0 * z1 - r1 * z0
    return prof if a > 0 else prof[::-1]


def rim_profile(bw, rb):
    """Closed drop-centre spoked-rim section (r, z), z = axle direction."""
    L = [(rb - R_UNDER, 0.0), (rb - R_UNDER, 0.0085), (rb - 0.0222, 0.0100), (rb - 0.0212, 0.0115),
         (rb - 0.0190, 0.0130), (rb - 0.0170, 0.0150)]
    if bw - 0.030 > 0.0165:
        L += [(rb - 0.0168, bw - 0.030)]
    L += [(rb - 0.0155, bw - 0.0265), (rb - 0.0068, bw - 0.0225), (rb - 0.0048, bw - 0.019),
          (rb - 0.0042, bw - 0.006), (rb - 0.0030, bw + 0.0025), (rb - 0.0012, bw + 0.0045),
          (rb + 0.0005, bw + 0.005), (rb + 0.0120, bw + 0.005)]
    rc, zc, ra = rb + 0.012, bw + 0.0025, 0.0025
    for k in range(1, 10):
        a = math.pi * k / 9
        L.append((rc + ra * math.sin(a), zc + ra * math.cos(a)))
    L += [(rb + 0.0028, bw), (rb + 0.0010, bw - 0.0006), (rb + 0.0002, bw - 0.0022), (rb - 0.0008, bw - 0.0150),
          (rb - 0.0002, bw - 0.0172), (rb + 0.0004, bw - 0.0185), (rb - 0.0004, bw - 0.0200),
          (rb - 0.0028, bw - 0.0218), (rb - 0.0060, bw - 0.0236), (rb - 0.0105, bw - 0.0258),
          (rb - 0.0125, bw - 0.0275), (rb - 0.0135, bw - 0.0292), (rb - 0.0135, 0.0)]
    right = [(r, -z) for (r, z) in reversed(L[1:-1])]
    return _ccw(L + right)


def lacing(rb, flanges, n=40, cross=3):
    """flanges: {+1: (z_f, r_hole, t), -1: (...)}.  Returns a list of spoke dicts (wheel frame)."""
    out = []
    r_u = rb - R_UNDER
    step = 360.0 / n
    for m in range(n):
        side = 1 if m % 2 == 0 else -1
        z_f, r_h, t = flanges[side]
        k = m // 2
        sg = 1 if k % 2 == 0 else -1
        th_m = math.radians(step / 2 + step * m)
        th_h = th_m + sg * math.radians(cross * 2 * step)
        head_in = sg > 0
        sz = 1 if z_f > 0 else -1
        if head_in:
            z_head, z_exit = z_f - sz * t / 2, z_f + sz * (t / 2 + 0.0022)
        else:
            z_head, z_exit = z_f + sz * t / 2, z_f - sz * (t / 2 + 0.0022)
        eh = Vector((math.cos(th_h), math.sin(th_h), 0.0))
        C = eh * r_h + Vector((0, 0, z_exit))
        Hh = eh * r_h + Vector((0, 0, z_head))
        N0 = Vector((r_u * math.cos(th_m), r_u * math.sin(th_m), 0.0045 * side))
        ndir = (C - N0).normalized()
        out.append(dict(side=side, H=Hh, C=C, N0=N0, ndir=ndir, z_head=z_head, z_exit=z_exit, th_m=th_m))
    return out


NIPPLE_PROF = [(0.0, -0.003), (0.0034, -0.003), (0.0034, 0.0004), (0.0031, 0.0010), (0.0026, 0.0016),
               (0.0026, 0.0095), (0.0024, 0.0105), (0.0021, 0.0112), (0.0, 0.0112)]


def rim_mb(name, bw, rb, spokes):
    mb = MB(name, ["alu_turned", "steel_polished"])
    bm = mb.bm
    lathe_bm(bm, rim_profile(bw, rb), 256, mb.mi("alu_turned"), closed=True)
    ms = mb.mi("steel_polished")
    for sp in spokes:
        lathe_bm(bm, NIPPLE_PROF, 12, ms, mat4=along(sp["N0"], sp["ndir"]))
    # valve stem between two nipples at theta = 0, through the rim centre band, pointing to the hub
    r_u = rb - R_UNDER
    ma = mb.mi("alu_turned")
    vprof = [(0.0, -0.008, ms), (0.0042, -0.008, ms), (0.0042, 0.0, ms), (0.0056, 0.0, ms), (0.0058, 0.0006, ms),
             (0.0058, 0.0034, ms), (0.0054, 0.0040, ms), (0.0038, 0.0040, ms), (0.0038, 0.0190, ms),
             (0.0044, 0.0190, ma), (0.0047, 0.0196, ma), (0.0047, 0.0300, ma), (0.0043, 0.0318, ma),
             (0.0030, 0.0326, ma), (0.0, 0.0328, ma)]
    lathe_bm(bm, vprof, 16, ms, mat4=along((r_u, 0.0, 0.0), (-1, 0, 0)))
    return mb


HEAD_PROF = [(0.0, 0.0017), (0.0016, 0.0016), (0.0027, 0.0011), (0.0032, 0.0004), (0.0032, 0.0), (0.0, 0.0)]


def spokes_mb(name, spokes, rad=0.002):
    mb = MB(name, ["steel_polished"])
    bm = mb.bm
    for sp in spokes:
        E = sp["N0"] + sp["ndir"] * 0.006
        path = fillet_poly([sp["H"], sp["C"], E], 0.0032, 6)
        sweep_bm(bm, path, rad, 10, 0, caps=True)
        dz = 1.0 if sp["z_head"] > sp["z_exit"] else -1.0
        lathe_bm(bm, HEAD_PROF, 14, 0, mat4=along(sp["H"], (0, 0, dz)))
    return mb


# ============================================================== hubs
def _mirror_loop(U):
    """U: upper half (r, z, mi) from the outer surface at z = 0 round to the bore at z = 0 (CCW).
    Returns the full symmetric closed loop with per-segment materials."""
    low = [(U[j][0], -U[j][1], U[j - 1][2]) for j in range(len(U) - 2, 0, -1)]
    return U + low


def front_hub_mb(name):
    mb = MB(name, ["alu_turned", "rubber", "steel_polished"])
    A, Rb, S = mb.mi("alu_turned"), mb.mi("rubber"), mb.mi("steel_polished")
    U = [(0.0300, 0.0000, A), (0.0296, 0.0060, A), (0.0284, 0.0150, A), (0.0280, 0.0200, A), (0.0271, 0.0207, A),
         (0.0271, 0.0233, A), (0.0280, 0.0240, A), (0.0286, 0.0305, A), (0.0298, 0.0335, A), (0.0322, 0.0358, A),
         (0.0352, 0.0371, A), (0.0372, 0.0375, A), (0.0490, 0.0375, A), (0.0500, 0.0385, A), (0.0500, 0.0415, A),
         (0.0490, 0.0425, A), (0.0362, 0.0425, A), (0.0334, 0.0434, A), (0.0313, 0.0452, A), (0.0306, 0.0478, A),
         (0.0311, 0.0498, A), (0.0328, 0.0511, A), (0.0350, 0.0515, A), (0.0452, 0.0515, A), (0.0465, 0.0528, A),
         (0.0465, 0.0572, A), (0.0452, 0.0585, A), (0.0242, 0.0585, A), (0.0228, 0.0591, A), (0.0222, 0.0600, A),
         (0.0201, 0.0600, A), (0.0195, 0.0594, A), (0.0195, 0.0580, Rb), (0.0168, 0.0585, Rb), (0.0143, 0.0580, S),
         (0.0140, 0.0592, S), (0.0113, 0.0592, S), (0.0110, 0.0588, A), (0.0110, 0.0000, A)]
    lathe_bm(mb.bm, _mirror_loop(U), 128, A, closed=True)
    return mb


def rear_hub_mb(name):
    mb = MB(name, ["alu_turned", "ano_orange_turned", "steel_polished"])
    A, O, S = mb.mi("alu_turned"), mb.mi("ano_orange_turned"), mb.mi("steel_polished")
    bm = mb.bm
    L = [(0.0000, -0.0955), (0.0072, -0.0955), (0.0078, -0.0962), (0.0080, -0.0985), (0.0150, -0.0985),
         (0.0168, -0.0981), (0.0178, -0.0971), (0.0180, -0.0960), (0.0180, -0.0690), (0.0336, -0.0690),
         (0.0352, -0.0684), (0.0362, -0.0672), (0.0376, -0.0660), (0.0402, -0.0645), (0.0436, -0.0612),
         (0.0466, -0.0570), (0.0486, -0.0528), (0.0496, -0.0498), (0.0505, -0.0482), (0.0522, -0.0475),
         (0.0570, -0.0475), (0.0580, -0.0465), (0.0580, -0.0435), (0.0570, -0.0425), (0.0462, -0.0425),
         (0.0430, -0.0410), (0.0410, -0.0385), (0.0402, -0.0355), (0.0400, -0.0180), (0.0392, -0.0172),
         (0.0392, -0.0148), (0.0400, -0.0140), (0.0400, 0.0080), (0.0392, 0.0088), (0.0392, 0.0112),
         (0.0400, 0.0120), (0.0400, 0.0255), (0.0410, 0.0285), (0.0430, 0.0310), (0.0462, 0.0325),
         (0.0570, 0.0325), (0.0580, 0.0335), (0.0580, 0.0365), (0.0570, 0.0375), (0.0468, 0.0375),
         (0.0457, 0.0385), (0.0455, 0.0398), (0.0455, 0.0448), (0.0448, 0.0455), (0.0380, 0.0455),
         (0.0380, 0.0558), (0.0465, 0.0562), (0.0500, 0.0564), (0.0510, 0.0572), (0.0510, 0.0592),
         (0.0500, 0.0600), (0.0212, 0.0600), (0.0200, 0.0590), (0.0200, 0.0420), (0.0000, 0.0420)]
    lathe_bm(bm, L, 160, A, closed=True)
    # centre-lock nut: washer flange + hex body + turned crown (orange anodised)
    lathe_bm(bm, [(0.0182, -0.0690), (0.0182, -0.0709), (0.0300, -0.0709), (0.0314, -0.0703),
                  (0.0316, -0.0696), (0.0312, -0.0690)], 96, O, closed=True)
    hexnut_prof_bm(bm, O, 0.0256, 0.0160, mat4=Matrix.Translation((0, 0, -0.0705)) @ Matrix.Rotation(math.pi, 4, "X"),
                   bevel=0.0009)
    lathe_bm(bm, [(0.0182, -0.0860), (0.0252, -0.0860), (0.0252, -0.0876), (0.0242, -0.0889), (0.0230, -0.0893),
                  (0.0182, -0.0893)], 96, O, closed=True)
    # retaining clip: leg through the stub cross-hole + loop resting on the nut crown
    zl, zc, rl = -0.0936, -0.0906, 0.0247
    path = [Vector((-0.0222, 0.0, zl)), Vector((0.0200, 0.0, zl)), Vector((0.0235, 0.0, zl + 0.0012)),
            Vector((rl, 0.0, zc))]
    for k in range(1, 41):
        a = math.radians(325.0) * k / 40
        path.append(Vector((rl * math.cos(a), rl * math.sin(a), zc)))
    a_end = math.radians(325.0)
    path.append(Vector((0.0285 * math.cos(a_end + 0.05), 0.0285 * math.sin(a_end + 0.05), zc - 0.0004)))
    path = fillet_poly(path, 0.002, 4)
    sweep_bm(bm, path, 0.00125, 10, S, caps=True)
    return mb


# ============================================================== floating discs
def disc_into(mb, z0, sgn, R_out, band_in, holes, hole_r, car_in, car_out, n_win, win_r0, win_r1, arm_w,
              bobbins, r_bob, hat=None, bolts=None, t=0.005):
    """One floating disc in the wheel frame, mid-plane at z = z0.  sgn = +1/-1 (which side, for the hat)."""
    T = Matrix.Translation((0, 0, z0))
    loops = [circle2d(0, 0, R_out, 288), circle2d(0, 0, band_in, 256)]
    loops += [circle2d(r * math.cos(a), r * math.sin(a), hole_r, 14) for (r, a) in holes]
    mb.add(plate_bm(loops, t, 0.0004, 1), "steel_turned", T)
    half_pitch = math.pi / n_win

    def win_half(r):
        return half_pitch - (arm_w / 2.0) / r

    loops = [circle2d(0, 0, car_out, 224), circle2d(0, 0, car_in, 128)]
    for k in range(n_win):
        loops.append(sq_sector(win_r0, win_r1, TAU * k / n_win + half_pitch, win_half, 56, 4.0))
    mb.add(plate_bm(loops, t * 0.96, 0.0005, 2), "ano_black", T)
    bob = [(0.0026, -0.0046), (0.0062, -0.0046), (0.0069, -0.0040), (0.0069, -0.0030), (0.0060, -0.0025),
           (0.0060, 0.0025), (0.0069, 0.0030), (0.0069, 0.0040), (0.0062, 0.0046), (0.0026, 0.0046)]
    mi_b = mb.mi("alu_machined")
    for a in bobbins:
        lathe_bm(mb.bm, bob, 24, mi_b, mat4=Matrix.Translation((r_bob * math.cos(a), r_bob * math.sin(a), z0)),
                 closed=True)
    if hat:
        prof = [(r, sgn * z) for (r, z) in hat]
        if sgn < 0:
            prof = prof[::-1]
        lathe_bm(mb.bm, prof, 128, mb.mi("ano_black"), closed=True)
    if bolts:
        n, rb_, zf, d, hh = bolts
        mi_s = mb.mi("steel_dark")
        for k in range(n):
            a = TAU * (k + 0.5) / n
            m = Matrix.Translation((rb_ * math.cos(a), rb_ * math.sin(a), sgn * zf))
            if sgn < 0:
                m = m @ Matrix.Rotation(math.pi, 4, "X")
            bolt_bm(mb.bm, mi_s, d=d, head_h=hh, mat4=m)


def front_discs_mb(name):
    mb = MB(name, ["steel_turned", "ano_black", "alu_machined", "steel_dark"])
    holes = []
    ng = 20
    for g in range(ng):
        a0 = TAU * g / ng
        for k in range(4):
            holes.append((0.1265 + 0.0075 * k, a0 + math.radians(2.7 * k)))
    bobbins = [TAU * g / ng + math.radians(9.0) for g in range(0, ng, 2)]
    hat = [(0.0262, 0.0585), (0.0462, 0.0585), (0.0478, 0.0590), (0.0490, 0.0603), (0.0490, 0.0770),
           (0.0450, 0.0770), (0.0450, 0.0645), (0.0440, 0.0630), (0.0425, 0.0625), (0.0262, 0.0625)]
    for sgn in (1, -1):
        disc_into(mb, sgn * common.DISC_F_Y, sgn, common.DISC_F_R, 0.1205, holes, 0.0029,
                  0.0446, 0.1185, 6, 0.0585, 0.1035, 0.0115, bobbins, 0.1195, hat=hat,
                  bolts=(6, 0.0360, 0.0625, 0.0062, 0.0045), t=common.DISC_T)
    return mb


REAR_DISC_Y = 0.048
REAR_DISC_R = 0.12


def rear_disc_mb(name):
    mb = MB(name, ["steel_turned", "ano_black", "alu_machined", "steel_dark"])
    holes = []
    ng = 16
    for g in range(ng):
        a0 = TAU * g / ng
        for k in range(3):
            holes.append((0.0995 + 0.0070 * k, a0 + math.radians(3.2 * k)))
    bobbins = [TAU * g / ng + math.radians(11.25) for g in range(0, ng, 2)]
    disc_into(mb, REAR_DISC_Y, 1, REAR_DISC_R, 0.0945, holes, 0.0025, 0.0385, 0.0925, 5, 0.0525, 0.0815, 0.0105,
              bobbins, 0.0935, hat=None, bolts=(6, 0.0435, REAR_DISC_Y + 0.0024, 0.0058, 0.0036), t=0.005)
    return mb


# ============================================================== rear drive (non-rotating, world axes, centre-relative)
M_XZ_PLATE = frame_matrix((0, 0, 0), (1, 0, 0), (0, 0, 1), (0, -1, 0))   # plate local XY -> world XZ


def xz_plate(mb, loops, y_c, thick, bevel, bres, mat):
    mb.add(plate_bm(loops, thick, bevel, bres), mat, Matrix.Translation((0, y_c, 0)) @ M_XZ_PLATE)


def polar(r, phi, y):
    return Vector((r * math.cos(phi), y, r * math.sin(phi)))


def radial_dir(phi):
    return Vector((math.cos(phi), 0.0, math.sin(phi)))


ARM_FLANGE_X = 0.075
CAL_PHI = math.radians(225.0)


def housing_mb(name):
    mb = MB(name, ["alu_cast", "alu_machined", "steel_dark", "rubber", "steel_polished"])
    bm = mb.bm
    C, M, D, Rb, S = (mb.mi(n) for n in mb.mats)
    prof = [(0.0000, 0.0740, C), (0.0380, 0.0740, C), (0.0380, 0.0705, Rb), (0.0440, 0.0701, Rb),
            (0.0500, 0.0705, M), (0.0540, 0.0705, M), (0.0540, 0.0663, M), (0.0553, 0.0650, M),
            (0.0792, 0.0650, M), (0.0832, 0.0664, C), (0.0850, 0.0694, C), (0.0850, 0.0962, C),
            (0.0844, 0.0972, C), (0.0834, 0.0976, C), (0.0834, 0.0980, M), (0.0846, 0.0982, M),
            (0.0850, 0.0988, M), (0.0850, 0.1006, M), (0.0844, 0.1013, M), (0.0834, 0.1015, C),
            (0.0640, 0.1015, C), (0.0608, 0.1021, C), (0.0582, 0.1040, C), (0.0565, 0.1072, C),
            (0.0556, 0.1118, C), (0.0550, 0.1180, C), (0.0550, 0.1500, C),
            (0.0541, 0.1549, C), (0.0517, 0.1584, C), (0.0482, 0.1600, C), (0.0000, 0.1600, C)]
    lathe_bm(bm, prof, 160, C, mat4=M_AXLE, closed=True)
    # machined outer cover + hex filler plug + low-head cover screws
    lathe_bm(bm, [(0.0, 0.1597), (0.0452, 0.1597), (0.0452, 0.1612), (0.0440, 0.1623), (0.0125, 0.1623),
                  (0.0114, 0.1628), (0.0, 0.1628)], 128, M, mat4=M_AXLE, closed=True)
    hexnut_prof_bm(bm, D, 0.0068, 0.0017, mat4=Matrix.Translation((0, 0.1627, 0)) @ Matrix.Rotation(-math.pi / 2, 4, "X"),
                   bevel=0.0004)
    for k in range(8):
        a = TAU * (k + 0.5) / 8
        bolt_bm(bm, D, d=0.0042, head_h=0.0017, mat4=along((0.0375 * math.cos(a), 0.1622, 0.0375 * math.sin(a)), (0, 1, 0)),
                segs=16)
    # neck to the swingarm flange (cast), flange with flats (machined), flange bolts
    nprof = [(0.0, 0.020), (0.0290, 0.020), (0.0290, 0.040), (0.0279, 0.0490), (0.0258, 0.0545), (0.0242, 0.0585),
             (0.0240, 0.0618), (0.0249, 0.0638), (0.0272, 0.0648), (0.0, 0.0648)]
    lathe_bm(bm, nprof, 64, C, mat4=frame_matrix((0, common.ARM_Y, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0)), closed=True)
    fl = []
    for i in range(96):
        a = TAU * i / 96
        p = (max(-0.0272, min(0.0272, 0.040 * math.cos(a))), 0.040 * math.sin(a))
        if not fl or abs(p[0] - fl[-1][0]) + abs(p[1] - fl[-1][1]) > 1e-6:
            fl.append(p)
    fb = plate_bm([fl], ARM_FLANGE_X - 0.0646, 0.0008, 2)
    mb.add(fb, "alu_machined", frame_matrix((0.0646 + (ARM_FLANGE_X - 0.0646) / 2, common.ARM_Y, 0), (0, 1, 0), (0, 0, 1),
                                            (1, 0, 0)))
    for a in (math.radians(v) for v in (35.0, 145.0, 215.0, 325.0)):
        p = (0.0646, common.ARM_Y + 0.032 * math.sin(a), 0.032 * math.cos(a))
        bolt_bm(bm, D, d=0.0062, head_h=0.0050, mat4=along(p, (-1, 0, 0)), segs=18)
    # bolted joint between crown-wheel case and pinion case (ring of screws on the shoulder face)
    for k in range(12):
        a = TAU * (k + 0.5) / 12
        if abs(math.atan2(math.sin(a), math.cos(a))) < math.radians(28.0):
            continue
        bolt_bm(bm, D, d=0.0050, head_h=0.0042, mat4=along((0.0742 * math.cos(a), 0.1015, 0.0742 * math.sin(a)), (0, 1, 0)),
                segs=16)
    # cooling fins round the rear/lower half of the pinion body
    for yf in (0.1225, 0.1325, 0.1425):
        lp = sq_sector(0.050, 0.0665, math.radians(180.0), lambda r: math.radians(76.0), 80, 6.0)
        xz_plate(mb, [lp], yf, 0.0036, 0.0011, 2, "alu_cast")
    # caliper lug (cast arm to the lower rear) with machined spot faces
    def lug_half(r):
        return (0.0105 + 0.0095 * (1.0 - smoothstep(0.079, 0.097, r))) / r
    xz_plate(mb, [sq_sector(0.076, 0.1180, CAL_PHI, lug_half, 72, 4.0)], 0.0790, 0.0140, 0.0016, 2, "alu_cast")
    for rr in (0.0985, 0.1115):
        lathe_bm(bm, [(0.0, 0.0855), (0.0058, 0.0855), (0.0058, 0.0866), (0.0, 0.0866)], 32, M,
                 mat4=Matrix.Translation(polar(rr, CAL_PHI, 0.0)) @ M_AXLE, closed=True)
        bolt_bm(bm, D, d=0.0062, head_h=0.0048, mat4=along(polar(rr, CAL_PHI, 0.0866), (0, 1, 0)), segs=18)
    # filler (top) and drain (bottom) plugs on radial bosses
    for phi, yb in ((math.radians(90.0), 0.0815), (math.radians(270.0), 0.0815)):
        d = radial_dir(phi)
        lathe_bm(bm, [(0.0, 0.070), (0.0098, 0.070), (0.0098, 0.0870), (0.0090, 0.0880), (0.0, 0.0880)], 32, C,
                 mat4=along(Vector((0, yb, 0)), d, (0, 1, 0)), closed=True)
        lathe_bm(bm, [(0.0, 0.0879), (0.0080, 0.0879), (0.0080, 0.0887), (0.0, 0.0887)], 32, M,
                 mat4=along(Vector((0, yb, 0)), d, (0, 1, 0)), closed=True)
        hexnut_prof_bm(bm, D, 0.0050, 0.0042, mat4=along(Vector((0, yb, 0)) + d * 0.0886, d, (0, 1, 0)), bevel=0.0005)
    # breather: boss, bent pipe, rubber cap
    phb = math.radians(78.0)
    d = radial_dir(phb)
    base = Vector((0, 0.1480, 0))
    lathe_bm(bm, [(0.0, 0.050), (0.0062, 0.050), (0.0062, 0.0590), (0.0055, 0.0596), (0.0, 0.0596)], 24, C,
             mat4=along(base, d, (0, 1, 0)), closed=True)
    p0 = base + d * 0.0590
    pts = [p0, p0 + d * 0.010, p0 + d * 0.014 + Vector((-0.010, 0.0, 0.0)), p0 + d * 0.014 + Vector((-0.022, 0.0, 0.0))]
    pts = fillet_poly(pts, 0.004, 5)
    sweep_bm(bm, pts, 0.0022, 12, S, caps=True)
    end = pts[-1]
    lathe_bm(bm, [(0.0, -0.006), (0.0034, -0.006), (0.0036, -0.0055), (0.0036, 0.0045), (0.0030, 0.0055), (0.0, 0.0057)],
             16, Rb, mat4=along(end, (-1, 0, 0)), closed=True)
    return mb


def caliper_mb(name):
    mb = MB(name, ["ano_black", "alu_machined", "steel_dark", "steel_polished", "rubber"])
    bm = mb.bm
    B, M, D, S, Rb = (mb.mi(n) for n in mb.mats)
    ph = CAL_PHI
    # outboard body (both pistons), bridge over the disc edge, inboard reaction finger
    xz_plate(mb, [sq_sector(0.0885, 0.1315, ph, lambda r: 0.0365 / r, 80, 5.0)], 0.0625, 0.0180, 0.0022, 2, "ano_black")
    xz_plate(mb, [sq_sector(0.1235, 0.1335, ph, lambda r: 0.0270 / r, 64, 5.0)], 0.0455, 0.0190, 0.0018, 2, "ano_black")
    xz_plate(mb, [sq_sector(0.0975, 0.1335, ph, lambda r: 0.0225 / r, 64, 5.0)], 0.0390, 0.0072, 0.0014, 2, "ano_black")
    # pads (backing plates)
    for yc in (0.0520, 0.0440):
        xz_plate(mb, [sq_sector(0.0975, 0.1185, ph, lambda r: 0.0235 / r, 48, 5.0)], yc, 0.0029, 0.0005, 1, "steel_dark")
    # machined piston caps on the outboard face
    for off in (-0.0190, 0.0190):
        a = ph + off / 0.1085
        lathe_bm(bm, [(0.0, 0.0732), (0.0040, 0.0732), (0.0040, 0.0726), (0.0068, 0.0726), (0.0080, 0.0731),
                      (0.0086, 0.0725), (0.0086, 0.0705), (0.0, 0.0705)], 40, M,
                 mat4=Matrix.Translation(polar(0.1085, a, 0.0)) @ M_AXLE, closed=True)
    # pad pin
    lathe_bm(bm, [(0.0, 0.0355), (0.0021, 0.0355), (0.0023, 0.0362), (0.0023, 0.0728), (0.0019, 0.0740), (0.0, 0.0740)],
             16, S, mat4=Matrix.Translation(polar(0.1275, ph + 0.012 / 0.1275, 0.0)) @ M_AXLE, closed=True)
    # bleed nipple on the upper end, pointing radially out
    a = ph - 0.0260 / 0.1315
    d = radial_dir(a)
    p = polar(0.1300, a, 0.0625)
    hexnut_prof_bm(bm, S, 0.0042, 0.0040, mat4=along(p, d, (0, 1, 0)), bevel=0.0004)
    lathe_bm(bm, [(0.0, 0.0035), (0.0028, 0.0035), (0.0028, 0.0085), (0.0, 0.0085)], 16, S, mat4=along(p, d, (0, 1, 0)),
             closed=True)
    lathe_bm(bm, [(0.0, 0.0070), (0.0036, 0.0070), (0.0038, 0.0078), (0.0038, 0.0128), (0.0030, 0.0136), (0.0, 0.0138)],
             16, Rb, mat4=along(p, d, (0, 1, 0)), closed=True)
    return mb


# ============================================================== build
def _wheel(col, root, tag, C, R, W, rim_w, rb, flanges, hub_fn, disc_fn, wmax, dmax):
    objs = []
    t = MB(tag + "_tyre", ["rubber_tire"])
    t.add(tyre_bm(R, W, rim_w / 2.0, rb, n_per=18, wmax=wmax, dmax=dmax), "rubber_tire")
    objs.append(t.finish(col, C, ROT_AXLE, 50.0))
    spokes = lacing(rb, flanges)
    objs.append(rim_mb(tag + "_rim", rim_w / 2.0, rb, spokes).finish(col, C, ROT_AXLE, 40.0))
    objs.append(spokes_mb(tag + "_spokes", spokes).finish(col, C, ROT_AXLE, 50.0))
    objs.append(hub_fn(tag + "_hub").finish(col, C, ROT_AXLE, 40.0))
    objs.append(disc_fn(tag + "_disc").finish(col, C, ROT_AXLE, 40.0))
    for o in objs:
        geo.parent(o, root)
    return objs


def build(col):
    fw = geo.empty("front_wheel", common.FW_C, col, size=0.36)
    rw = geo.empty("rear_wheel", common.RW_C, col, size=0.36)
    rd = geo.empty("rear_drive", common.RW_C, col, size=0.12)
    _wheel(col, fw, "fw", common.FW_C, common.FW_R, common.FW_W, common.FW_RIM_W, common.FW_RIM_R,
           {1: (0.040, 0.0455, 0.005), -1: (-0.040, 0.0455, 0.005)}, front_hub_mb, front_discs_mb, 0.0055, 0.0035)
    _wheel(col, rw, "rw", common.RW_C, common.RW_R, common.RW_W, common.RW_RIM_W, common.RW_RIM_R,
           {1: (0.035, 0.0535, 0.005), -1: (-0.045, 0.0535, 0.005)}, rear_hub_mb, rear_disc_mb, 0.0065, 0.0040)
    for mb in (housing_mb("rd_housing"), caliper_mb("rd_caliper")):
        o = mb.finish(col, common.RW_C, (0, 0, 0), 40.0)
        geo.parent(o, rd)
    return {"front_wheel": fw, "rear_wheel": rw, "rear_drive": rd}
