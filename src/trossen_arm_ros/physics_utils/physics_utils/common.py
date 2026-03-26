import numpy as np
from enum import Enum


class Player(Enum):
    PLAYER = 0
    OPPONENT = 1
    

table_x = 1.37 + 0.9
table_length = 2.74
table_width = 1.525
table_height = 0.76
table_thickness = 0.1
def in_table(xyz, margin=0.01):
    x, y, z = xyz
    return (hit_plane - margin <= x <= table_x + table_length / 2 + margin and
            -table_width / 2 - margin <= y <= table_width / 2 + margin and
            z >= table_height - margin)

table_surface_z = table_height + table_thickness / 2
hit_plane = table_x - table_length / 2  - 0.3
landing_pos = np.array([table_x + table_length / 2, 0.0, table_surface_z])

cam_z_error = 0.04
compute_time_adjust = 0.08