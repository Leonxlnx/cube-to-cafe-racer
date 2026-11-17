"""Small procedural-modelling helpers (Blender 5.2, bmesh).  All sizes in metres.

Every function returns a bpy object already linked into `col` (a collection) with
identity transform unless stated otherwise.  Geometry is created in object space.
"""
import math
import bpy
import bmesh
from mathutils import Vector, Matrix, Euler

DEFAULT_COL = None


def _col(col):
    return col or DEFAULT_COL or bpy.context.scene.collection


def obj_from_bm(name, bm, col=None, smooth=None):
    me = bpy.data.meshes.new(name)
    bm.normal_update()
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    _col(col).objects.link(ob)
    if smooth is not None:
        set_smooth(ob, smooth)
    return ob


def obj_from_pydata(name, verts, faces, col=None, smooth=None):
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in verts], [], [tuple(f) for f in faces])
    me.update()
    ob = bpy.data.objects.new(name, me)
    _col(col).objects.link(ob)
    if smooth is not None:
        set_smooth(ob, smooth)
    return ob


def set_smooth(ob, angle_deg=35.0):
    """Smooth shading with sharp edges above angle_deg (None/0 -> flat)."""
    me = ob.data
    if not angle_deg:
        me.shade_flat()
        return ob
    me.shade_smooth()
    try:
        me.set_sharp_from_angle(angle=math.radians(angle_deg))
    except Exception:
        pass
    return ob


def bevel_sharp_angle(bsegs, base=35.0):
    """Sharp-edge threshold that keeps the big faces of a bevelled solid flat (no pillow shading)."""
    return min(base, 0.9 * 90.0 / (bsegs + 1))


def set_mat(ob, mat):
    ob.data.materials.clear()
    ob.data.materials.append(mat)
    return ob


def empty(name, loc=(0, 0, 0), col=None, size=0.05):
    e = bpy.data.objects.new(name, None)
    e.empty_display_size = size
    e.location = loc
    _col(col).objects.link(e)
    return e


def parent(child, par, keep_world=True):
    """Parent child to par. keep_world=True keeps the child's current world transform."""
    if keep_world:
        bpy.context.view_layer.update()
        mw = child.matrix_world.copy()
        child.parent = par
        child.matrix_parent_inverse = par.matrix_world.inverted()
        child.matrix_world = mw
    else:
        child.parent = par
        child.matrix_parent_inverse = Matrix.Identity(4)
    return child


def place(ob, loc=None, rot=None, scale=None):
    if loc is not None:
        ob.location = loc
    if rot is not None:
        ob.rotation_euler = Euler(rot, "XYZ")
    if scale is not None:
        ob.scale = scale if hasattr(scale, "__len__") else (scale, scale, scale)
    return ob


def apply_transform(ob):
    """Bake the object's transform into its mesh data (object becomes identity)."""
    bpy.context.view_layer.update()
    ob.data.transform(ob.matrix_basis)
    ob.matrix_basis = Matrix.Identity(4)
    ob.data.update()
    return ob


def transform_mesh(ob, loc=(0, 0, 0), rot=(0, 0, 0), scale=(1, 1, 1)):
    """Transform the mesh DATA (keeps object at identity). rot = XYZ euler radians."""
    m = Matrix.Translation(Vector(loc)) @ Euler(rot, "XYZ").to_matrix().to_4x4() @ Matrix.Diagonal((scale[0], scale[1], scale[2], 1.0))
    ob.data.transform(m)
    ob.data.update()
    return ob


def bevel_mod(ob, width=0.001, segments=2, angle_deg=30.0, harden=True):
    m = ob.modifiers.new("Bevel", "BEVEL")
    m.width = width
    m.segments = segments
    m.limit_method = "ANGLE"
    m.angle_limit = math.radians(angle_deg)
    try:
        m.harden_normals = harden
    except Exception:
        pass
    return m


def subsurf_mod(ob, levels=2):
    m = ob.modifiers.new("Subsurf", "SUBSURF")
    m.levels = levels
    m.render_levels = levels
    return m


def apply_mods(ob):
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    old = ob.data
    ob.modifiers.clear()
    ob.data = me
    if old.users == 0:
        bpy.data.meshes.remove(old)
    return ob


def join(objs, name=None):
    """Join mesh objects (bakes their transforms). Returns the joined object (the first one)."""
    objs = [o for o in objs if o is not None]
    if not objs:
        return None
    dg = bpy.context.evaluated_depsgraph_get()
    bpy.context.view_layer.update()
    bm = bmesh.new()
    mats = []
    for o in objs:
        ev = o.evaluated_get(dg)
        me = ev.to_mesh()
        tmp = bmesh.new()
        tmp.from_mesh(me)
        ev.to_mesh_clear()
        # material slots
        slot_map = {}
        for i, s in enumerate(o.material_slots):
            if s.material not in mats:
                mats.append(s.material)
            slot_map[i] = mats.index(s.material)
        for f in tmp.faces:
            f.material_index = slot_map.get(f.material_index, 0)
        tmp.transform(o.matrix_world)
        me2 = bpy.data.meshes.new("tmpjoin")
        tmp.to_mesh(me2)
        tmp.free()
        bm.from_mesh(me2)
        bpy.data.meshes.remove(me2)
    first = objs[0]
    col = first.users_collection[0] if first.users_collection else _col(None)
    me = bpy.data.meshes.new(name or first.name)
    bm.to_mesh(me)
    bm.free()
    res = bpy.data.objects.new(name or first.name, me)
    col.objects.link(res)
    for mt in mats:
        me.materials.append(mt)
    # keep smooth flags as they were per face (from_mesh keeps them); sharp edges are kept too
    for o in objs:
        d = o.data
        bpy.data.objects.remove(o, do_unlink=True)
        if d.users == 0:
            bpy.data.meshes.remove(d)
    return res


# ----------------------------------------------------------------------------- primitives
def box(name, sx, sy, sz, bevel=0.0, segs=2, col=None, mat=None, loc=(0, 0, 0)):
    """Axis-aligned box centred at loc (in mesh data), optional real bevel."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=(sx, sy, sz), verts=bm.verts)
    if bevel > 0:
        bmesh.ops.bevel(bm, geom=bm.edges[:], offset=bevel, segments=segs, affect="EDGES", profile=0.5)
    bmesh.ops.translate(bm, vec=loc, verts=bm.verts)
    ob = obj_from_bm(name, bm, col, smooth=bevel_sharp_angle(segs) if bevel > 0 else None)
    if mat:
        set_mat(ob, mat)
    return ob


def cyl(name, r, depth, segs=48, bevel=0.0, bsegs=2, axis="Z", col=None, mat=None, loc=(0, 0, 0), r2=None):
    """Capped cylinder (or cone frustum with r2 = top radius) centred at loc, along axis."""
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=segs, radius1=r,
                          radius2=r if r2 is None else r2, depth=depth)
    if bevel > 0:
        rim = [e for e in bm.edges if len(e.link_faces) == 2 and any(len(f.verts) > 4 for f in e.link_faces)]
        bmesh.ops.bevel(bm, geom=rim, offset=bevel, segments=bsegs, affect="EDGES", profile=0.5)
    if axis == "X":
        bmesh.ops.rotate(bm, verts=bm.verts, cent=(0, 0, 0), matrix=Matrix.Rotation(math.pi / 2, 3, "Y"))
    elif axis == "Y":
        bmesh.ops.rotate(bm, verts=bm.verts, cent=(0, 0, 0), matrix=Matrix.Rotation(-math.pi / 2, 3, "X"))
    bmesh.ops.translate(bm, vec=loc, verts=bm.verts)
    ob = obj_from_bm(name, bm, col, smooth=bevel_sharp_angle(bsegs, 40.0) if bevel > 0 else 40.0)
    if mat:
        set_mat(ob, mat)
    return ob


def lathe(name, profile, segs=64, axis="Z", col=None, mat=None, closed=False, smooth=40.0):
    """Revolve a profile [(r, z), ...] around the axis.  Points with r == 0 become poles.
    closed=True connects the last profile point back to the first (ring-shaped section)."""
    bm = bmesh.new()
    n = len(profile)
    rings = []
    for (r, z) in profile:
        if r <= 1e-9:
            rings.append([bm.verts.new((0.0, 0.0, z))])
        else:
            rings.append([bm.verts.new((r * math.cos(2 * math.pi * i / segs), r * math.sin(2 * math.pi * i / segs), z))
                          for i in range(segs)])
    pairs = list(zip(range(n - 1), range(1, n)))
    if closed:
        pairs.append((n - 1, 0))
    for a, b_ in pairs:
        ra, rb = rings[a], rings[b_]
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
            try:
                bm.faces.new(f)
            except ValueError:
                pass
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    if axis == "X":
        bmesh.ops.rotate(bm, verts=bm.verts, cent=(0, 0, 0), matrix=Matrix.Rotation(math.pi / 2, 3, "Y"))
    elif axis == "Y":
        bmesh.ops.rotate(bm, verts=bm.verts, cent=(0, 0, 0), matrix=Matrix.Rotation(-math.pi / 2, 3, "X"))
    ob = obj_from_bm(name, bm, col, smooth=smooth)
    if mat:
        set_mat(ob, mat)
    return ob


def prism(name, pts2d, depth, bevel=0.0, bsegs=2, plane="XZ", col=None, mat=None, center=True):
    """Extrude a 2D polygon (CCW list of (u, v)) into a plate of thickness `depth`.
    plane="XY": polygon in XY, thickness along Z.  "XZ": polygon in XZ, thickness along Y.
    "YZ": polygon in YZ, thickness along X.  Plate is centred on the thickness axis if center."""
    bm = bmesh.new()
    vs = [bm.verts.new((u, v, 0.0)) for (u, v) in pts2d]
    f = bm.faces.new(vs)
    bm.normal_update()
    if f.normal.z < 0:
        bmesh.ops.reverse_faces(bm, faces=[f])
    ext = bmesh.ops.extrude_face_region(bm, geom=[f])
    nv = [g for g in ext["geom"] if isinstance(g, bmesh.types.BMVert)]
    bmesh.ops.translate(bm, vec=(0, 0, depth), verts=nv)
    if center:
        bmesh.ops.translate(bm, vec=(0, 0, -depth / 2.0), verts=bm.verts)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    if bevel > 0:
        bmesh.ops.bevel(bm, geom=bm.edges[:], offset=bevel, segments=bsegs, affect="EDGES", profile=0.5)
    if plane == "XZ":      # (u, v, w) -> (u, -w, v)
        bmesh.ops.rotate(bm, verts=bm.verts, cent=(0, 0, 0), matrix=Matrix.Rotation(math.pi / 2, 3, "X"))
    elif plane == "YZ":    # (u, v, w) -> (w, u, v)
        m = Matrix(((0, 0, 1), (1, 0, 0), (0, 1, 0)))
        bmesh.ops.transform(bm, matrix=m.to_4x4(), verts=bm.verts)
    ob = obj_from_bm(name, bm, col, smooth=bevel_sharp_angle(bsegs) if bevel > 0 else None)
    if mat:
        set_mat(ob, mat)
    return ob


def tube(name, pts, r, segs=16, fillet=0.0, fillet_res=8, closed=False, caps=True, col=None, mat=None,
         radii=None):
    """Round tube through 3D points. fillet > 0 rounds the corners (poly line -> arcs).
    radii: optional per-input-point radius scale (only without fillet)."""
    P = [Vector(p) for p in pts]
    if fillet > 0 and len(P) > 2:
        Q = [P[0]] if not closed else []
        rng = range(1, len(P) - 1) if not closed else range(len(P))
        for i in rng:
            a, b_, c = P[i - 1], P[i], P[(i + 1) % len(P)]
            d1, d2 = (a - b_), (c - b_)
            l1, l2 = d1.length, d2.length
            if l1 < 1e-9 or l2 < 1e-9:
                Q.append(b_)
                continue
            d1.normalize(); d2.normalize()
            ang = d1.angle(d2)
            if ang > math.pi - 1e-3:
                Q.append(b_)
                continue
            t = min(fillet / math.tan(ang / 2.0), l1 * 0.49, l2 * 0.49)
            p1, p2 = b_ + d1 * t, b_ + d2 * t
            for k in range(fillet_res + 1):
                u = k / fillet_res
                # quadratic bezier is a good enough arc approximation
                Q.append((1 - u) ** 2 * p1 + 2 * u * (1 - u) * b_ + u ** 2 * p2)
        if not closed:
            Q.append(P[-1])
        P = Q
        radii = None
    cu = bpy.data.curves.new(name + "_cu", "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = r
    cu.bevel_resolution = max(1, segs // 4 - 1)
    cu.use_fill_caps = caps and not closed
    sp = cu.splines.new("POLY")
    sp.points.add(len(P) - 1)
    for i, p in enumerate(P):
        sp.points[i].co = (p.x, p.y, p.z, 1.0)
        if radii is not None:
            sp.points[i].radius = radii[i]
    sp.use_cyclic_u = closed
    tmp = bpy.data.objects.new(name + "_tmp", cu)
    _col(col).objects.link(tmp)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(tmp.evaluated_get(dg))
    bpy.data.objects.remove(tmp, do_unlink=True)
    bpy.data.curves.remove(cu)
    me.name = name
    ob = bpy.data.objects.new(name, me)
    _col(col).objects.link(ob)
    set_smooth(ob, 60.0)
    if mat:
        set_mat(ob, mat)
    return ob


def torus(name, R, r, segs=48, rsegs=16, axis="Z", col=None, mat=None, loc=(0, 0, 0)):
    prof = [(R + r * math.cos(2 * math.pi * k / rsegs), r * math.sin(2 * math.pi * k / rsegs)) for k in range(rsegs)]
    ob = lathe(name, prof, segs, axis, col, mat, closed=True, smooth=80.0)
    transform_mesh(ob, loc=loc)
    return ob


def bolt(name, d=0.008, head_h=None, kind="socket", col=None, mat=None, washer=True):
    """Bolt head sitting on the XY plane, pointing up +Z (head top at z = head_h).
    kind: "socket" (cap screw with hex recess), "hex" (hex head), "button" (domed)."""
    head_h = head_h or d * 0.9
    bm = bmesh.new()
    if kind == "hex":
        bmesh.ops.create_cone(bm, cap_ends=True, segments=6, radius1=d * 0.9, radius2=d * 0.9, depth=head_h)
        bmesh.ops.translate(bm, vec=(0, 0, head_h / 2), verts=bm.verts)
    else:
        rr = d * 0.78
        bmesh.ops.create_cone(bm, cap_ends=True, segments=24, radius1=rr, radius2=rr * (0.8 if kind == "button" else 1.0),
                              depth=head_h * (0.6 if kind == "button" else 1.0))
        hh = head_h * (0.6 if kind == "button" else 1.0)
        bmesh.ops.translate(bm, vec=(0, 0, hh / 2), verts=bm.verts)
        # hex recess: inset + push down the top face
        top = max(bm.faces, key=lambda f: f.calc_center_median().z)
        res = bmesh.ops.inset_region(bm, faces=[top], thickness=rr * 0.42)
        bmesh.ops.translate(bm, vec=(0, 0, -hh * 0.55), verts=top.verts)
    if washer:
        w = bmesh.new()
        bmesh.ops.create_cone(w, cap_ends=True, segments=24, radius1=d * 1.15, radius2=d * 1.15, depth=d * 0.14)
        bmesh.ops.translate(w, vec=(0, 0, d * 0.07), verts=w.verts)
        bmesh.ops.translate(bm, vec=(0, 0, d * 0.14), verts=bm.verts)
        me = bpy.data.meshes.new("w")
        w.to_mesh(me)
        w.free()
        bm.from_mesh(me)
        bpy.data.meshes.remove(me)
    ob = obj_from_bm(name, bm, col, smooth=40.0)
    if mat:
        set_mat(ob, mat)
    return ob


def copy_obj(ob, name=None, col=None, linked=True):
    """Duplicate an object (linked=True shares mesh data: cheap instancing)."""
    c = ob.copy()
    if not linked:
        c.data = ob.data.copy()
    if name:
        c.name = name
    (col or (ob.users_collection[0] if ob.users_collection else _col(None))).objects.link(c)
    return c


def polar_copies(ob, n, axis="Z", center=(0, 0, 0), start=0.0, include_first=True):
    """Return n linked copies rotated about an axis through center (first one is ob itself)."""
    out = [ob] if include_first else []
    base = ob.matrix_world.copy()
    for i in range(1, n):
        c = copy_obj(ob)
        rot = Matrix.Translation(Vector(center)) @ Matrix.Rotation(start + 2 * math.pi * i / n, 4, axis) @ Matrix.Translation(-Vector(center))
        c.matrix_world = rot @ base
        out.append(c)
    return out


def bounds(objs):
    """World-space bounding box (min Vector, max Vector) of mesh objects."""
    bpy.context.view_layer.update()
    mn = Vector((1e9, 1e9, 1e9))
    mx = Vector((-1e9, -1e9, -1e9))
    for o in objs:
        if o.type != "MESH":
            continue
        for c in o.bound_box:
            w = o.matrix_world @ Vector(c)
            mn = Vector((min(mn.x, w.x), min(mn.y, w.y), min(mn.z, w.z)))
            mx = Vector((max(mx.x, w.x), max(mx.y, w.y), max(mx.z, w.z)))
    return mn, mx


def tri_count(objs):
    dg = bpy.context.evaluated_depsgraph_get()
    n = 0
    for o in objs:
        if o.type != "MESH":
            continue
        ev = o.evaluated_get(dg)
        me = ev.to_mesh()
        n += sum(len(p.vertices) - 2 for p in me.polygons)
        ev.to_mesh_clear()
    return n
