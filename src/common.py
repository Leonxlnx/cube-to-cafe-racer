"""Shared dimensions / hard points for the cube -> cafe racer film.

Coordinates: metres, X forward (bike nose), Y left, Z up, floor at z = 0.
The cube is centred on the origin: x, y in [-H, H], z in [0, S].
All bike hard points below are given for the FINAL (unfolded, ride height) pose
in world coordinates ("chassis frame" == world in the final pose).
"""
import math

# ------------------------------------------------------------------ cube
S = 1.10            # cube edge
H = S / 2.0
T_PLATE = 0.005     # plate thickness
SEAM = 0.0010       # gap between neighbouring plates
DROP = 0.06         # chassis sits this much lower while folded inside the cube
HOVER = 0.015       # wheels hover this much above the floor while folded

# ------------------------------------------------------------------ wheels
RW_C = (-0.784, 0.0, 0.33)   # rear wheel centre
RW_R = 0.33                  # rear tyre outer radius
RW_W = 0.19                  # rear tyre width
RW_RIM_R = 0.216             # 17" rim bead seat radius
RW_RIM_W = 0.140

FW_C = (0.69, 0.0, 0.32)     # front wheel centre
FW_R = 0.32
FW_W = 0.125
FW_RIM_R = 0.216
FW_RIM_W = 0.090

DISC_F_R = 0.155             # front brake discs (two): outer radius
DISC_F_Y = 0.079             # |y| of the disc mid-plane measured from the wheel centre plane
DISC_T = 0.005

# rear: single sided, everything non-rotating lives on the LEFT (+Y) of the wheel
ARM_Y = 0.13                 # y of the swingarm centre line (left of wheel centre plane)

# ------------------------------------------------------------------ front end
RAKE = math.radians(25.0)
# fork frame F: origin on the steering axis at the TOP face of the upper triple clamp.
# F axes: Z = steering axis (up/back), X = forward, perpendicular to the axis, Y = left.
F_O = (0.361, 0.0, 0.950)
FORK_OFFSET = 0.032          # fork tube axes sit at F.x = +0.032
FORK_HALF_SPAN = 0.105       # fork tube axes at F.y = +-0.105
FORK_LEN = 0.71              # front wheel centre in F = (0.032, 0, -0.71) at full extension
FORK_OUTER_R = 0.027         # upper (outer) tube radius, USD fork
FORK_INNER_R = 0.0215        # chrome stanchion radius
FORK_OUTER_BOTTOM = -0.44    # F.z of the lower end of the outer tubes
CLAMP_UP_Z = (-0.030, 0.0)   # upper triple clamp occupies this F.z range
CLAMP_LO_Z = (-0.215, -0.175)
STEM_R = 0.016
FORK_COMPRESS_FOLDED = 0.18  # how far the lowers slide up (F +Z) while folded
# rake hinge (transverse axis) of the head carrier, in F coordinates
HINGE_C_F = (-0.070, 0.0, -0.045)

# ------------------------------------------------------------------ rear arm
PIVOT = (-0.29, 0.0, 0.405)  # swingarm pivot axis (transverse, along Y)
LINK1 = 0.315                # inner link (pivot block -> elbow)
LINK2 = 0.185                # outer link (elbow -> wheel axle)

# ------------------------------------------------------------------ engine envelope
ENG_X = (-0.250, 0.200)
ENG_Y = (-0.200, 0.200)
ENG_Z = (0.190, 0.670)

# ------------------------------------------------------------------ bodywork (final)
TANK_X = (-0.32, 0.24)       # top plate of the tank
TANK_TOP_Z = 0.93
SEAT_X = (-0.74, -0.32)
SEAT_Z = (0.79, 0.85)
KEEL_Z = (0.10, 0.16)
KEEL_X = (-0.26, 0.29)


def fork_axes():
    """Unit vectors of the fork frame F expressed in world (final pose)."""
    s, c = math.sin(RAKE), math.cos(RAKE)
    ex = (c, 0.0, s)       # F.x  (forward, slightly up)
    ey = (0.0, 1.0, 0.0)   # F.y
    ez = (-s, 0.0, c)      # F.z  (steering axis, up and back)
    return ex, ey, ez


def fork_to_world(p):
    ex, ey, ez = fork_axes()
    return tuple(F_O[i] + p[0] * ex[i] + p[1] * ey[i] + p[2] * ez[i] for i in range(3))
