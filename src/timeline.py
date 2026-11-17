"""Animation timeline (seconds). No Blender imports, so tools outside Blender can read it.

Film time = T - 0.4 during the transformation (see shots.py).  The big music hit / hero cut is at
film 25.0, the headlight comes on at film 23.93 (a beat).
"""

TL = dict(
    breath=(5.00, 5.80),
    door_F=(6.00, 8.10), door_B=(6.35, 8.45),
    lid_up=(8.20, 9.20),
    rows=(8.80, 10.90),
    rear=(11.00, 13.40),
    carr=(11.60, 13.60),
    rake=(13.20, 15.00),
    steer=(13.90, 15.40),
    clip=(15.00, 16.00),
    ext=(15.90, 16.75),          # suspension extends until the tyres touch
    land=16.75,                  # touch-down, chassis starts to rise
    stand=(16.80, 17.45),
    tail_hem=(14.60, 15.40), tail_side=(16.90, 17.90), tail_down=(17.10, 18.10), tail_close=(18.00, 18.80),
    tank_back=(16.90, 17.85), tank_hem=(17.10, 17.95), tank_fold=(17.90, 19.05), tank_down=(18.95, 19.90),
    nose=(20.10, 20.60), cap=(20.70, 21.50),
    keel_L=(19.80, 22.00), keel_R=(21.50, 23.40),
    tray_L=(20.40, 22.20), tray_R=(20.80, 22.60),
    side=(21.90, 23.50),
    lights=24.33,
    end=25.00,
)
