"""The cube skin: 48 wall plates (4 walls x 4 columns x 3 rows) and 15 lid plates (tank net + tail net).

Every plate is a machined panel: a thin brushed outer skin with a perimeter frame and an orthogrid of
stiffening ribs milled into the inner face, flush screws on the outside, slider shoes / hinge knuckles
on the inside.  Plate local axes: X = brushing direction, Z = outward normal.
"""
import math
import bpy
import bmesh
from mathutils import Matrix, Vector

import common as C
import geo
import mats
from rigbase import frame, rot_about, T, R

t = C.T_PLATE
S = C.S
H = C.H
ROWH = (S - t) / 3.0          # wall row height (lid sits on top of the walls)
COLW_SIDE = S / 4.0           # side wall column width
COLW_END = (S - 2 * t) / 4.0  # front / rear wall column width (between the side walls)
BEV = 0.0007
SKIN = 0.0021                 # skin thickness left by the pockets
FRAME_W = 0.011               # perimeter frame width
RIB_W = 0.0026
CELL = 0.046                  # orthogrid pitch

_mesh_cache = {}


def _inset(poly, d):
    """Offset a CCW polygon inward by d (miter joins)."""
    n = len(poly)
    out = []
    for i in range(n):
        p0 = Vector(poly[i - 1]); p1 = Vector(poly[i]); p2 = Vector(poly[(i + 1) % n])
        e1 = (p1 - p0).normalized(); e2 = (p2 - p1).normalized()
        n1 = Vector((-e1.y, e1.x)); n2 = Vector((-e2.y, e2.x))      # inward normals of a CCW polygon
        a1 = p0 + n1 * d; a2 = p1 + n2 * d
        den = e1.x * e2.y - e1.y * e2.x
        if abs(den) < 1e-9:
            out.append(tuple(p1 + n1 * d))
        else:
            tt = ((a2.x - a1.x) * e2.y - (a2.y - a1.y) * e2.x) / den
            out.append(tuple(a1 + e1 * tt))
    return out


def _ccw(poly):
    area = sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
               for i in range(len(poly)))
    return poly if area > 0 else poly[::-1]


def _prism_bm(bm, pts, z0, z1, mat_index):
    """Add a closed prism (CCW polygon pts between z0 and z1) to bm."""
    lo = [bm.verts.new((p[0], p[1], z0)) for p in pts]
    hi = [bm.verts.new((p[0], p[1], z1)) for p in pts]
    n = len(pts)
    faces = []
    try:
        faces.append(bm.faces.new(hi))
        faces.append(bm.faces.new(lo[::-1]))
    except ValueError:
        pass
    for i in range(n):
        j = (i + 1) % n
        try:
            faces.append(bm.faces.new((lo[i], lo[j], hi[j], hi[i])))
        except ValueError:
            pass
    for f in faces:
        f.material_index = mat_index
    return faces


def _disc_bm(bm, cx, cy, z, r, mat_index, segs=14, star=False):
    vs = []
    for k in range(segs):
        a = 2 * math.pi * k / segs
        rr = r * (0.55 if (star and k % 2) else 1.0)
        vs.append(bm.verts.new((cx + rr * math.cos(a), cy + rr * math.sin(a), z)))
    f = bm.faces.new(vs)
    f.material_index = mat_index
    return f


def _cyl_bm(bm, p0, p1, r, mat_index, segs=10):
    """Capped cylinder between two 3D points."""
    p0 = Vector(p0); p1 = Vector(p1)
    ax = (p1 - p0).normalized()
    u = ax.orthogonal().normalized()
    v = ax.cross(u)
    a = [bm.verts.new(p0 + (u * math.cos(2 * math.pi * k / segs) + v * math.sin(2 * math.pi * k / segs)) * r) for k in range(segs)]
    b = [bm.verts.new(p1 + (u * math.cos(2 * math.pi * k / segs) + v * math.sin(2 * math.pi * k / segs)) * r) for k in range(segs)]
    fs = [bm.faces.new(a[::-1]), bm.faces.new(b)]
    for k in range(segs):
        j = (k + 1) % segs
        fs.append(bm.faces.new((a[k], a[j], b[j], b[k])))
    for f in fs:
        f.material_index = mat_index
    return fs


def _line_poly_spans(poly, axis, c):
    """Intervals where the line (x = c if axis == 0 else y = c) lies inside the polygon."""
    hits = []
    n = len(poly)
    o = 1 - axis
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if (a[axis] - c) * (b[axis] - c) < 0:
            u = (c - a[axis]) / (b[axis] - a[axis])
            hits.append(a[o] + u * (b[o] - a[o]))
    hits.sort()
    return [(hits[k], hits[k + 1]) for k in range(0, len(hits) - 1, 2)]


def _skin_with_hole(bm, poly, hole, z0, z1):
    """Skin slab with a round hole (no boolean): two triangulated caps + outer and inner walls."""
    cx, cy, r = hole
    segs = 72
    n = len(poly)
    layers = []
    for z in (z0, z1):
        vo = [bm.verts.new((p[0], p[1], z)) for p in poly]
        vi = [bm.verts.new((cx + r * math.cos(2 * math.pi * k / segs), cy + r * math.sin(2 * math.pi * k / segs), z))
              for k in range(segs)]
        eo = [bm.edges.new((vo[i], vo[(i + 1) % n])) for i in range(n)]
        ei = [bm.edges.new((vi[i], vi[(i + 1) % segs])) for i in range(segs)]
        bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=False, edges=eo + ei)
        layers.append((vo, vi))
    (vo0, vi0), (vo1, vi1) = layers
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((vo0[i], vo0[j], vo1[j], vo1[i]))
    for i in range(segs):
        j = (i + 1) % segs
        bm.faces.new((vi0[j], vi0[i], vi1[i], vi1[j]))


def panel_mesh(poly, name, knuckles=(), shoes=True, hole=None):
    """Machined panel mesh for a CCW polygon (plate-local 2D).  Material slots:
    0 alu_plate (skin), 1 alu_machined (frame, ribs, bosses), 2 steel_dark (screws), 3 ano_orange (shoes),
    4 steel_polished (hinge knuckles).  knuckles: list of ((x0, y0), (x1, y1)) segments on the inner face."""
    bm = bmesh.new()
    zo = t / 2.0
    zi = -t / 2.0
    zs = zo - SKIN
    # skin (bevelled)
    if hole is None:
        skin = _prism_bm(bm, poly, zs, zo, 0)
        edges = set()
        for f in skin:
            for e in f.edges:
                edges.add(e)
        bmesh.ops.bevel(bm, geom=list(edges), offset=BEV, segments=2, affect="EDGES", profile=0.5)
    else:
        _skin_with_hole(bm, poly, hole, zs, zo)
    for f in bm.faces:
        f.material_index = 0
    # perimeter frame
    inner = _inset(poly, FRAME_W)
    n = len(poly)
    outer_in = _inset(poly, 0.0006)
    for i in range(n):
        j = (i + 1) % n
        _prism_bm(bm, [outer_in[i], outer_in[j], inner[j], inner[i]], zi, zs + 0.0002, 1)
    # ribs (orthogrid) clipped to the inner polygon
    xs = [p[0] for p in inner]
    ys = [p[1] for p in inner]
    for axis, lo, hi in ((0, min(xs), max(xs)), (1, min(ys), max(ys))):
        span = hi - lo
        k = max(1, int(round(span / CELL)))
        for q in range(1, k):
            c = lo + span * q / k
            spans = _line_poly_spans(inner, axis, c)
            if hole is not None:
                hc = hole[axis]
                ho = hole[1 - axis]
                rr = hole[2] + 0.005
                if abs(c - hc) < rr:
                    dd = math.sqrt(rr * rr - (c - hc) ** 2)
                    cut = []
                    for (a, b) in spans:
                        if a < ho - dd:
                            cut.append((a, min(b, ho - dd)))
                        if b > ho + dd:
                            cut.append((max(a, ho + dd), b))
                    spans = cut
            for (a, b) in spans:
                if b - a < 0.004:
                    continue
                if axis == 0:
                    pts = [(c - RIB_W / 2, a), (c + RIB_W / 2, a), (c + RIB_W / 2, b), (c - RIB_W / 2, b)]
                else:
                    pts = [(a, c - RIB_W / 2), (b, c - RIB_W / 2), (b, c + RIB_W / 2), (a, c + RIB_W / 2)]
                _prism_bm(bm, pts, zi + 0.0008, zs + 0.0002, 1)
    # screws: flush heads outside, bosses inside
    sc_pts = []
    sc_poly = _inset(poly, 0.017)
    for i_ in range(len(sc_poly)):
        q0 = Vector(sc_poly[i_]); q1 = Vector(sc_poly[(i_ + 1) % len(sc_poly)])
        ln = (q1 - q0).length
        k_ = max(1, int(math.ceil(ln / 0.125)))
        for j_ in range(k_):
            sc_pts.append(tuple(q0.lerp(q1, j_ / k_)))
    for (sx, sy) in sc_pts:
        _disc_bm(bm, sx, sy, zo + 0.00012, 0.0036, 4, 16)
        _disc_bm(bm, sx, sy, zo + 0.00022, 0.0022, 2, 12, star=True)
        boss = [(sx + 0.0062 * math.cos(2 * math.pi * k / 10), sy + 0.0062 * math.sin(2 * math.pi * k / 10)) for k in range(10)]
        _prism_bm(bm, boss, zi + 0.0003, zs + 0.0002, 1)
        hx = [(sx + 0.0034 * math.cos(2 * math.pi * k / 6), sy + 0.0034 * math.sin(2 * math.pi * k / 6)) for k in range(6)]
        _prism_bm(bm, hx, zi - 0.0004, zi + 0.0006, 2)
    # slider shoes on the inside (wall plates)
    if shoes:
        for sgn in (1, -1):
            ex = (max(xs) if sgn > 0 else min(xs))
            for cy in (min(ys) + 0.035, max(ys) - 0.035):
                x0, x1 = sorted((ex, ex - sgn * 0.020))
                _prism_bm(bm, [(x0, cy - 0.011), (x1, cy - 0.011), (x1, cy + 0.011), (x0, cy + 0.011)], zi - 0.0002, zi + 0.0026, 3)
    # filler neck flange around the hole
    if hole is not None:
        cx_, cy_, r_ = hole
        segs_ = 48
        for k in range(segs_):
            a0 = 2 * math.pi * k / segs_
            a1 = 2 * math.pi * (k + 1) / segs_
            q = [(cx_ + r_ * math.cos(a0), cy_ + r_ * math.sin(a0)), (cx_ + (r_ + 0.0045) * math.cos(a0), cy_ + (r_ + 0.0045) * math.sin(a0)),
                 (cx_ + (r_ + 0.0045) * math.cos(a1), cy_ + (r_ + 0.0045) * math.sin(a1)), (cx_ + r_ * math.cos(a1), cy_ + r_ * math.sin(a1))]
            _prism_bm(bm, q, zi - 0.004, zs + 0.0002, 1)
    # hinge knuckles
    for (p0, p1) in knuckles:
        _cyl_bm(bm, (p0[0], p0[1], zi - 0.0012), (p1[0], p1[1], zi - 0.0012), 0.0031, 4, 10)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    me = bpy.data.meshes.new(name)
    bm.normal_update()
    bm.to_mesh(me)
    bm.free()
    me.shade_flat()
    for mn in ("alu_plate", "alu_machined", "steel_dark", "ano_orange", "steel_polished"):
        me.materials.append(mats.get(mn))
    return me


def new_plate(name, w, h, col):
    key = ("rect", round(w, 5), round(h, 5))
    me = _mesh_cache.get(key)
    if me is None:
        poly = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
        me = panel_mesh(poly, "plate_%dx%d" % (round(w * 1000), round(h * 1000)))
        _mesh_cache[key] = me
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    return ob


# ----------------------------------------------------------------------------- walls
def build_walls(col):
    """Returns list of dicts: obj, side (+1 left / -1 right), leaf (bool), end ('F'/'B'/None),
    c (column index along x on its side wall after the doors are folded back: 0 rear .. 3 front),
    r (row 0 bottom .. 2 top), M0 (closed world matrix), hinge (x, y) and swing (radians) for leaves."""
    out = []
    for side in (1, -1):
        zax = (0, side, 0)
        for c in range(4):
            xc = -H + (c + 0.5) * COLW_SIDE
            for r in range(3):
                zc = (r + 0.5) * ROWH
                ob = new_plate("wall_%s_c%d_r%d" % ("L" if side > 0 else "R", c, r), COLW_SIDE - C.SEAM, ROWH - C.SEAM, col)
                M0 = frame((xc, side * (H - t / 2), zc), (1, 0, 0), zax)
                out.append(dict(obj=ob, side=side, leaf=False, end=None, c=c, r=r, M0=M0))
    for end in ("F", "B"):
        sx = 1 if end == "F" else -1
        for j in range(4):                       # columns along y, j = 0 at y = -H + t
            yc = -(H - t) + (j + 0.5) * COLW_END
            side = 1 if yc > 0 else -1
            for r in range(3):
                zc = (r + 0.5) * ROWH
                ob = new_plate("door_%s_j%d_r%d" % (end, j, r), COLW_END - C.SEAM, ROWH - C.SEAM, col)
                M0 = frame((sx * (H - t / 2), yc, zc), (0, 1, 0), (sx, 0, 0))
                hinge = (sx * H, side * H)
                # front-left: +270, front-right: -270, rear-left: -270, rear-right: +270
                swing = math.radians(270.0) * sx * side
                xa = abs(yc)
                cc = (2 if xa < COLW_END else 3) if end == "F" else (1 if xa < COLW_END else 0)
                out.append(dict(obj=ob, side=side, leaf=True, end=end, c=cc, r=r, M0=M0, hinge=hinge, swing=swing))
    return out


# ----------------------------------------------------------------------------- lid
# widths (y extents, |y| from .. to) of the two nets
TANK_BANDS = dict(T=(0.0, 0.10), S=(0.10, 0.19), V=(0.19, 0.37), U=(0.37, 0.55))
TAIL_BANDS = dict(TT=(0.0, 0.09), TS=(0.09, 0.27), TB1=(0.27, 0.415), TB2=(0.415, 0.55))
X_SPLIT = -0.11       # tail zone | tank zone (at y = 0)
X_NOSE = 0.45         # T | N
CAP_POS = (0.11, 0.0)   # filler cap centre in T-local coordinates
CAP_R = 0.036
CHEV_K = 0.45          # chevron slope of the tail | tank seam: x_cut(y) = X_SPLIT + CHEV_K * |y|


def xcut(y):
    return X_SPLIT + CHEV_K * abs(y)


def _knuckle_segments(p0, p1, parity, n=9, gap=0.0016):
    """Alternate hinge knuckles along the line p0 -> p1 (lid coordinates); parity 0 / 1 picks every other one."""
    p0 = Vector(p0); p1 = Vector(p1)
    d = (p1 - p0)
    L = d.length
    u = d / L
    a0 = p0 + u * (0.07 * L)
    seg = 0.86 * L / n
    out = []
    for k in range(parity, n, 2):
        s0 = a0 + u * (k * seg + gap / 2)
        s1 = a0 + u * ((k + 1) * seg - gap / 2)
        out.append(((s0.x, s0.y), (s1.x, s1.y)))
    return out


def build_lid(col):
    """Returns dict name -> dict(obj, cx, cy).  Names: T, N, S+, S-, V+, V-, U+, U-, TT, TS+.. TB2-.
    The tail | tank seam is a chevron, so the plates are quads / pentagons.  Hinge knuckles are added
    along the fold lines (alternating between the two plates of each hinge)."""
    P = {}
    hinges = []
    for s, sg in ((1, "+"), (-1, "-")):
        hinges.append(("T", "S" + sg, (xcut(0.10) + 0.01, s * 0.10), (X_NOSE - 0.01, s * 0.10)))
        hinges.append(("S" + sg, "V" + sg, (xcut(0.19) + 0.01, s * 0.19), (H - 0.012, s * 0.19)))
        hinges.append(("V" + sg, "U" + sg, (xcut(0.37) + 0.01, s * 0.37), (H - 0.012, s * 0.37)))
        hinges.append(("TT", "TS" + sg, (-H + 0.012, s * 0.09), (xcut(0.09) - 0.01, s * 0.09)))
        hinges.append(("TS" + sg, "TB1" + sg, (-H + 0.012, s * 0.27), (xcut(0.27) - 0.01, s * 0.27)))
    hinges.append(("T", "N", (X_NOSE, -0.088), (X_NOSE, 0.088)))

    def add(name, poly, x0, x1, y0, y1):
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        poly = _ccw(poly)
        pts = _inset(poly, C.SEAM / 2)
        loc = [(px - cx, py - cy) for (px, py) in pts]
        kn = []
        for (a, b, p0, p1) in hinges:
            if name in (a, b):
                for (q0, q1) in _knuckle_segments(p0, p1, 0 if name == a else 1):
                    kn.append(((q0[0] - cx, q0[1] - cy), (q1[0] - cx, q1[1] - cy)))
        hole = (CAP_POS[0], CAP_POS[1], CAP_R + 0.0006) if name == "T" else None
        me = panel_mesh(loc, "lid_" + name, knuckles=kn, shoes=False, hole=hole)
        ob = bpy.data.objects.new("lid_" + name, me)
        col.objects.link(ob)
        P[name] = dict(obj=ob, cx=cx, cy=cy)

    add("T", [(xcut(0.10), -0.10), (X_NOSE, -0.10), (X_NOSE, 0.10), (xcut(0.10), 0.10), (xcut(0.0), 0.0)],
        X_SPLIT, X_NOSE, -0.10, 0.10)
    add("N", [(X_NOSE, -0.10), (H, -0.10), (H, 0.10), (X_NOSE, 0.10)], X_NOSE, H, -0.10, 0.10)
    add("TT", [(-H, -0.09), (xcut(0.09), -0.09), (xcut(0.0), 0.0), (xcut(0.09), 0.09), (-H, 0.09)],
        -H, X_SPLIT, -0.09, 0.09)
    for sg, s in (("+", 1), ("-", -1)):
        for nm in ("S", "V", "U"):
            a, b = TANK_BANDS[nm]
            y0, y1 = (a, b) if s > 0 else (-b, -a)
            add(nm + sg, [(xcut(y0), y0), (H, y0), (H, y1), (xcut(y1), y1)], X_SPLIT, H, y0, y1)
        for nm in ("TS", "TB1", "TB2"):
            a, b = TAIL_BANDS[nm]
            y0, y1 = (a, b) if s > 0 else (-b, -a)
            add(nm + sg, [(-H, y0), (xcut(y0), y0), (xcut(y1), y1), (-H, y1)], -H, X_SPLIT, y0, y1)

    return P


def build_cap(col):
    """Flush filler cap (origin at the plate mid-plane of T).  Knurled skirt, seal ring and bayonet lugs
    are hidden below the plate until the cap pops up."""
    h_top = t / 2
    prof = [(0.0, h_top), (CAP_R - 0.0012, h_top), (CAP_R, h_top - 0.0012), (CAP_R, -0.004),
            (CAP_R - 0.002, -0.006), (CAP_R - 0.002, -0.018), (CAP_R - 0.006, -0.020), (0.0, -0.020)]
    cap = geo.lathe("fuel_cap", prof, 96, "Z", col, mats.get("alu_plate"))
    bm = bmesh.new()
    n = 72
    for i in range(n):
        a = 2 * math.pi * i / n
        m = Matrix.Rotation(a, 4, "Z") @ Matrix.Translation((CAP_R - 0.0007, 0, -0.0105))
        bmesh.ops.create_cube(bm, size=1.0, matrix=m @ Matrix.Diagonal((0.0022, 0.0018, 0.0085, 1.0)))
    knurl = geo.obj_from_bm("fuel_cap_knurl", bm, col, smooth=None)
    geo.set_mat(knurl, mats.get("alu_machined"))
    parts = [cap, knurl]
    seal = geo.torus("cap_seal", CAP_R - 0.0035, 0.0016, 64, 8, "Z", col, mats.get("ano_orange"), loc=(0, 0, -0.0165))
    parts.append(seal)
    for i in range(3):
        a = 2 * math.pi * i / 3
        lugo = geo.box("cap_lug", 0.010, 0.006, 0.004, bevel=0.0008, col=col, mat=mats.get("steel_polished"))
        lugo.matrix_world = Matrix.Rotation(a, 4, "Z") @ Matrix.Translation((CAP_R - 0.004, 0, -0.019))
        parts.append(lugo)
    cap = geo.join(parts, "fuel_cap")
    return cap
