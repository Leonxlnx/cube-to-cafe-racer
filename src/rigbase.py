"""Math, easing and a tiny transform tree used by the film rig."""
import math
import bpy
from mathutils import Matrix, Vector, Quaternion

I4 = Matrix.Identity(4)


# ----------------------------------------------------------------------------- easing
def clamp01(x):
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def ph(t, t0, t1):
    """Linear phase 0..1 of t inside [t0, t1]."""
    if t1 <= t0:
        return 1.0 if t >= t1 else 0.0
    return clamp01((t - t0) / (t1 - t0))


def smooth(u):
    u = clamp01(u)
    return u * u * (3.0 - 2.0 * u)


def smoother(u):
    u = clamp01(u)
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def ease_in(u, p=2.0):
    return clamp01(u) ** p


def ease_out(u, p=2.0):
    return 1.0 - (1.0 - clamp01(u)) ** p


def ease_io(u, p=2.0):
    u = clamp01(u)
    if u < 0.5:
        return 0.5 * (2 * u) ** p
    return 1.0 - 0.5 * (2 * (1 - u)) ** p


def back_out(u, s=1.2):
    """Overshoots a little and settles (mechanical 'arrive')."""
    u = clamp01(u) - 1.0
    return u * u * ((s + 1.0) * u + s) + 1.0


def settle(u, bounces=2.0, damp=5.0, amp=1.0):
    """0 -> 1 with a damped oscillation around 1 (use for things that land and ring)."""
    u = clamp01(u)
    if u >= 1.0:
        return 1.0
    return 1.0 - amp * math.exp(-damp * u) * math.cos(2 * math.pi * bounces * u) * (1.0 - u)


def mech(u, accel=0.22, over=0.035, ring=1.6):
    """Servo-like move: quick smooth acceleration, firm stop with a tiny overshoot + ring-out.
    Returns 0..1 (momentarily slightly above 1 near the end)."""
    u = clamp01(u)
    w = 0.86                       # fraction of the time used for the main move
    if u < w:
        return smoother(u / w) * (1.0 + over) if False else smoother(u / w)
    v = (u - w) / (1.0 - w)
    return 1.0 + over * math.sin(math.pi * 2 * ring * v) * math.exp(-4.0 * v) * (1.0 - v)


def lerp(a, b, u):
    return a + (b - a) * u


def vlerp(a, b, u):
    return Vector(a).lerp(Vector(b), u)


# ----------------------------------------------------------------------------- matrices
def T(x=0.0, y=0.0, z=0.0):
    if isinstance(x, (Vector, tuple, list)):
        return Matrix.Translation(Vector(x))
    return Matrix.Translation(Vector((x, y, z)))


def R(axis, angle):
    return Matrix.Rotation(angle, 4, axis)


def rot_about(point, axis, angle):
    p = Vector(point)
    if isinstance(axis, str):
        r = Matrix.Rotation(angle, 4, axis)
    else:
        r = Matrix.Rotation(angle, 4, Vector(axis))
    return Matrix.Translation(p) @ r @ Matrix.Translation(-p)


def frame(origin, x, z):
    """4x4 with given origin, X axis x and Z axis z (orthonormalised, Y = Z cross X)."""
    zx = Vector(z).normalized()
    xx = Vector(x)
    xx = (xx - zx * xx.dot(zx)).normalized()
    yy = zx.cross(xx)
    m = Matrix((
        (xx.x, yy.x, zx.x, origin[0]),
        (xx.y, yy.y, zx.y, origin[1]),
        (xx.z, yy.z, zx.z, origin[2]),
        (0.0, 0.0, 0.0, 1.0)))
    return m


def mlerp(A, B, u):
    """Interpolate two rigid transforms (lerp position, slerp rotation)."""
    if u <= 0.0:
        return A.copy()
    if u >= 1.0:
        return B.copy()
    la, qa, _ = A.decompose()
    lb, qb, _ = B.decompose()
    q = qa.slerp(qb, u)
    return Matrix.Translation(la.lerp(lb, u)) @ q.to_matrix().to_4x4()


def path(keys, t):
    """keys: list of (time, Matrix, ease_fn or None). Returns interpolated matrix at t."""
    if t <= keys[0][0]:
        return keys[0][1].copy()
    for i in range(1, len(keys)):
        t0, m0 = keys[i - 1][0], keys[i - 1][1]
        t1, m1 = keys[i][0], keys[i][1]
        if t <= t1:
            e = keys[i][2] if len(keys[i]) > 2 and keys[i][2] else smooth
            return mlerp(m0, m1, e(ph(t, t0, t1)))
    return keys[-1][1].copy()


# ----------------------------------------------------------------------------- transform tree
class Node:
    """An Empty in Blender plus a python-side local matrix (parent inverse is identity)."""
    registry = []

    def __init__(self, name, parent=None, local=None, col=None):
        self.name = name
        self.parent = parent
        self.local = (local or I4).copy()
        self.rest = self.local.copy()
        e = bpy.data.objects.new(name, None)
        e.empty_display_size = 0.03
        (col or bpy.context.scene.collection).objects.link(e)
        if parent is not None:
            e.parent = parent.ob
            e.matrix_parent_inverse = Matrix.Identity(4)
        e.matrix_basis = self.local
        self.ob = e
        Node.registry.append(self)

    def set(self, m):
        self.local = m
        self.ob.matrix_basis = m

    def world(self):
        if self.parent is None:
            return self.local.copy()
        return self.parent.world() @ self.local

    def attach(self, obj, world=None):
        """Parent a Blender object to this node keeping (or assigning) its world matrix.
        Call while the tree is in the pose the object was modelled for."""
        mw = (world if world is not None else obj.matrix_world).copy()
        obj.parent = self.ob
        obj.matrix_parent_inverse = Matrix.Identity(4)
        obj.matrix_basis = self.world().inverted() @ mw
        return obj


def ik2(a, b, l1, l2, pole):
    """Elbow position of a two-link arm from a to b (lengths l1, l2) bending toward `pole` direction.
    If the distance exceeds l1 + l2 the arm is returned straight (stretched proportionally)."""
    a = Vector(a)
    b = Vector(b)
    d = b - a
    dist = d.length
    if dist < 1e-9:
        return a + Vector(pole).normalized() * l1
    u = d / dist
    if dist >= l1 + l2:
        return a + u * (dist * l1 / (l1 + l2))
    if dist <= abs(l1 - l2):
        dist = abs(l1 - l2) + 1e-6
    x = (l1 * l1 - l2 * l2 + dist * dist) / (2.0 * dist)
    h2 = max(l1 * l1 - x * x, 0.0)
    h = math.sqrt(h2)
    p = Vector(pole)
    p = p - u * p.dot(u)
    if p.length < 1e-9:
        p = u.orthogonal()
    p.normalize()
    return a + u * x + p * h


def aim(a, b, up=(0, 0, 1)):
    """Matrix at a with local X pointing to b, local Z as close to `up` as possible."""
    a = Vector(a)
    x = (Vector(b) - a)
    if x.length < 1e-9:
        x = Vector((1, 0, 0))
    x.normalize()
    upv = Vector(up)
    z = upv - x * upv.dot(x)
    if z.length < 1e-6:
        z = x.orthogonal()
    z.normalize()
    y = z.cross(x)
    return Matrix((
        (x.x, y.x, z.x, a.x),
        (x.y, y.y, z.y, a.y),
        (x.z, y.z, z.z, a.z),
        (0, 0, 0, 1)))
