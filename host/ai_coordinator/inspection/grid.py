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


def segment_distance(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    norm = dx*dx+dy*dy
    t = 0 if norm == 0 else max(0, min(1, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/norm))
    return math.hypot(point[0]-a[0]-t*dx, point[1]-a[1]-t*dy)


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


def build(objects, vehicle_radius_m, *, corridor=None, corridor_half_width_m=None):
    if not finite(vehicle_radius_m) or not 0 < vehicle_radius_m <= 1:
        raise ValueError('Measured vehicle swept radius required')
    raw, cost = bytearray(PLANE_BYTES), bytearray(PLANE_BYTES)
    for item in objects:
        if not item['confirmed']:
            continue
        radius = item['radius_m']+item['xy_error_m']
        circle(raw, item['xy_m'], radius)
        circle(cost, item['xy_m'], radius+vehicle_radius_m)
    if corridor is not None:
        if not 2 <= len(corridor) <= 1024 or not finite(corridor_half_width_m) or corridor_half_width_m <= vehicle_radius_m:
            raise ValueError('Return corridor must contain measured vehicle footprint')
        for point in corridor:
            cell(point)
        centre_margin = corridor_half_width_m-vehicle_radius_m
        half_diagonal = CELL_M/math.sqrt(2)
        import numpy as np
        allowed = np.zeros((SIZE, SIZE), dtype=bool)
        radius = centre_margin-half_diagonal
        for a, b in zip(corridor, corridor[1:]):
            lo = [max(0, math.floor((min(a[k], b[k])-centre_margin-ORIGIN_M)/CELL_M)) for k in (0, 1)]
            hi = [min(SIZE, math.ceil((max(a[k], b[k])+centre_margin-ORIGIN_M)/CELL_M)) for k in (0, 1)]
            if hi[0] <= lo[0] or hi[1] <= lo[1] or radius < 0:
                continue
            y, x = np.mgrid[lo[1]:hi[1], lo[0]:hi[0]]
            dx, dy = b[0]-a[0], b[1]-a[1]
            px = ORIGIN_M+(x+.5)*CELL_M-a[0]
            py = ORIGIN_M+(y+.5)*CELL_M-a[1]
            norm = dx*dx+dy*dy
            t = 0 if norm == 0 else np.clip((px*dx+py*dy)/norm, 0, 1)
            allowed[lo[1]:hi[1], lo[0]:hi[0]] |= (px-t*dx)**2+(py-t*dy)**2 <= radius*radius
        excluded = np.packbits(~allowed.reshape(-1), bitorder='little').tobytes()
        cost = bytearray(a | b for a, b in zip(cost, excluded))
    payload = bytes(raw+cost)
    return {'raw': bytes(raw), 'cost': bytes(cost), 'crc32': zlib.crc32(payload) & 0xffffffff,
            'resolution_mm': 50, 'origin_mm': [-6400, -6400], 'size': [256, 256]}
