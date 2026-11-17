"""Material swatch scene (also a smoke test for geo/mats/studio)."""
import math
import geo
import mats


def build(col):
    names = mats.names()
    out = {}
    n = len(names)
    cols = 8
    for i, nm in enumerate(names):
        r, c = divmod(i, cols)
        x = (c - (cols - 1) / 2) * 0.16
        y = (r - 1.5) * 0.2
        if nm in ("alu_plate", "alu_brushed", "ano_orange", "steel_brushed", "titanium"):
            o = geo.box(nm, 0.13, 0.005, 0.13, bevel=0.0008, col=col)
            o.location = (x, y, 0.075)
            o.rotation_euler = (0, 0, math.radians(20))
        elif nm.endswith("turned") or nm in ("ano_titan",):
            o = geo.cyl(nm, 0.06, 0.02, 64, bevel=0.002, col=col)
            o.location = (x, y, 0.07)
            o.rotation_euler = (math.radians(70), 0, math.radians(20))
        else:
            o = geo.lathe(nm, [(0, -0.06)] + [(0.06 * math.cos(math.radians(a)), 0.06 * math.sin(math.radians(a)))
                                             for a in range(-80, 81, 10)] + [(0, 0.06)], 48, col=col, smooth=80)
            o.location = (x, y, 0.065)
        geo.set_mat(o, mats.get(nm))
        if nm.startswith("emit"):
            mats.set_emit(o, 1.0)
        out[nm] = o
    return out
