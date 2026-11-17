"""Envelope check for parts.engine (used from preview_part --post)."""
import bpy


def check(res):
    root = res["engine"]
    bpy.context.view_layer.update()
    viol = {}
    emn = [9, 9, 9]
    emx = [-9, -9, -9]
    for o in root.children:
        if o.type != "MESH":
            continue
        mw = o.matrix_world
        ex = any(t in o.name for t in ("headers", "titanium", "ti_blued", "steel_brushed"))
        mn = [9, 9, 9]
        mx = [-9, -9, -9]
        for v in o.data.vertices:
            p = mw @ v.co
            for i in range(3):
                mn[i] = min(mn[i], p[i])
                mx[i] = max(mx[i], p[i])
            if p.x > 0.262:
                viol["x>0.262"] = viol.get("x>0.262", 0) + 1
            if p.z > 0.67:
                viol["z>0.67"] = viol.get("z>0.67", 0) + 1
            if p.x < -0.2501 and not (-0.31 <= p.y <= -0.205 and 0.15 <= p.z <= 0.26):
                viol["x<-0.25 outside exhaust window " + o.name] = viol.get("x<-0.25 outside exhaust window " + o.name, 0) + 1
            if abs(p.y) > 0.31 or p.z < 0.15:
                viol["outer"] = viol.get("outer", 0) + 1
            if abs(p.y) < 0.19 and p.z < 0.17:
                viol["keel"] = viol.get("keel", 0) + 1
            if not ex and p.y > -0.2005 and not (p.x > 0.2 and p.z < 0.3):
                for i in range(3):
                    emn[i] = min(emn[i], p[i])
                    emx[i] = max(emx[i], p[i])
                if p.x > 0.2001 or abs(p.y) > 0.2001 or p.z < 0.1899:
                    key = "engine proper outside envelope: " + o.name
                    viol[key] = viol.get(key, 0) + 1
        print("CHECK %-28s min (%.4f %.4f %.4f) max (%.4f %.4f %.4f) tris %d" % (
            o.name, mn[0], mn[1], mn[2], mx[0], mx[1], mx[2], sum(len(p.vertices) - 2 for p in o.data.polygons)))
    print("CHECK engine-proper bounds min (%.4f %.4f %.4f) max (%.4f %.4f %.4f)" % tuple(emn + emx))
    print("CHECK violations:", viol if viol else "none")
