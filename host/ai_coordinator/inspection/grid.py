"""RK-generated raw/cost planes. x advances first; bit0 is the lowest x."""
import math
import zlib
from .calibration import finite

SIZE, CELL_M, ORIGIN_M = 256, .05, -6.4
PLANE_BYTES = SIZE*SIZE//8


def cell(xy):
    if len(xy) != 2 or not all(finite(v) and -6.4 <= v < 6.4 for v in xy):
        raise ValueError('Point outside navigation map')
    return tuple(math.floor((v-ORIGIN_M)/CELL_M) for v in xy)


def bit(plane, x, y):
    address = y*SIZE+x
    return (plane[address//8] >> (address % 8)) & 1


def set_bit(plane, x, y):
    address = y*SIZE+x
    plane[address//8] |= 1 << (address % 8)


def circle(plane, xy, radius):
    cell(xy)
    if not finite(radius) or radius <= 0:
        raise ValueError('Positive measured radius required')
    lo = [max(0, math.floor((v-radius-ORIGIN_M)/CELL_M)) for v in xy]
    hi = [min(SIZE-1, math.floor((v+radius-ORIGIN_M)/CELL_M)) for v in xy]
    for y in range(lo[1], hi[1]+1):
        for x in range(lo[0], hi[0]+1):
            # Circle intersects this square, not merely its centre.
            near_x = max(ORIGIN_M+x*CELL_M, min(xy[0], ORIGIN_M+(x+1)*CELL_M))
            near_y = max(ORIGIN_M+y*CELL_M, min(xy[1], ORIGIN_M+(y+1)*CELL_M))
            if math.hypot(near_x-xy[0], near_y-xy[1]) <= radius:
                set_bit(plane, x, y)


def build(objects, vehicle_radius_m):
    if not finite(vehicle_radius_m) or not 0 < vehicle_radius_m <= 1:
        raise ValueError('Measured vehicle swept radius required')
    raw, cost = bytearray(PLANE_BYTES), bytearray(PLANE_BYTES)
    for item in objects:
        if not item['confirmed']:
            continue
        radius = item['radius_m']+item['xy_error_m']
        circle(raw, item['xy_m'], radius)
        circle(cost, item['xy_m'], radius+vehicle_radius_m)
    payload = bytes(raw+cost)
    return {'raw': bytes(raw), 'cost': bytes(cost), 'crc32': zlib.crc32(payload) & 0xffffffff,
            'resolution_mm': 50, 'origin_mm': [-6400, -6400], 'size': [256, 256]}
