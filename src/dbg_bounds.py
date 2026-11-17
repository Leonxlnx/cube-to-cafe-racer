"""Print world bounds of sub-assemblies at a given T (default 0) and flag what leaves the cube."""
import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy
from mathutils import Vector
import studio, film, geo, common as C

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
T = float(argv[0]) if argv else 0.0
studio.clear_scene()
st = studio.build_open()
fm = film.Film()
fm.pose(T)
bpy.context.view_layer.update()


def kids(ob):
    out = []
    for c in ob.children:
        out.append(c)
        out += kids(c)
    return out


lim = C.H - C.T_PLATE
for node in film.Node.registry:
    direct = [c for c in node.ob.children if c.type == "MESH"]
    # include agent roots (empties that are not rig nodes)
    names = {n.ob.name for n in film.Node.registry}
    for c in node.ob.children:
        if c.type == "EMPTY" and c.name not in names:
            direct += [k for k in kids(c) if k.type == "MESH"]
    if not direct:
        continue
    mn, mx = geo.bounds(direct)
    bad = []
    if mx.x > lim or mn.x < -lim:
        bad.append("X")
    if mx.y > lim or mn.y < -lim:
        bad.append("Y")
    if mx.z > C.S - C.T_PLATE:
        bad.append("Z")
    print("BND %-8s x[%6.3f %6.3f] y[%6.3f %6.3f] z[%6.3f %6.3f] %s" % (node.name, mn.x, mx.x, mn.y, mx.y, mn.z, mx.z, " ".join(bad)))
    if bad and node.name not in ("TN", "TT") and not node.name.startswith(("p", "tp")):
        for o in direct:
            a, b = geo.bounds([o])
            if b.x > lim or a.x < -lim or b.y > lim or a.y < -lim or b.z > C.S - C.T_PLATE:
                print("      out: %-28s x[%6.3f %6.3f] y[%6.3f %6.3f] z[%6.3f %6.3f]" % (o.name, a.x, b.x, a.y, b.y, a.z, b.z))
