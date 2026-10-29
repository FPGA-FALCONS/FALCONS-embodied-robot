"""Bounded static object catalogue; misses never erase a confirmed obstacle."""
from copy import deepcopy
import math
from .calibration import finite

GROUND_ANCHORS = {
    'tape': 'box_center',
    'screwdriver': 'box_center',
    'ad_calcium_milk': 'bottom_center',
    'nutri_express': 'bottom_center',
}


def ground_anchor(detection):
    x1,y1,x2,y2=detection['bbox']
    v=(y1+y2)/2 if GROUND_ANCHORS[detection['class']]=='box_center' else y2-1
    return dict(u=(x1+x2)/2,v=v)


class Objects:
    def __init__(self, confirmations=3, merge_m=.15, limit=128):
        if confirmations < 1 or merge_m <= 0 or not 1 <= limit <= 512:
            raise ValueError('Invalid object catalogue bounds')
        self.confirmations, self.merge_m, self.limit = confirmations, merge_m, limit
        self.items = []; self.last_sequence = -1; self.epoch = None

    def observe(self, detections, sequence, epoch, now, *, consecutive=True):
        if not epoch or type(sequence) is not int or sequence <= self.last_sequence:
            raise ValueError('Fresh observation sequence and epoch required')
        if self.epoch not in (None, epoch):
            raise ValueError('Reset catalogue explicitly after localization reset')
        if not finite(now) or len(detections) > 128:
            raise ValueError('Invalid observation batch')
        clean = []
        for d in detections:
            xy = d.get('xy_m')
            if (not isinstance(d.get('class'), str) or not d['class'] or not isinstance(xy, (list, tuple)) or
                    len(xy) != 2 or not all(finite(v) and -6.4 <= v < 6.4 for v in xy) or
                    not finite(d.get('radius_m')) or not 0 < d['radius_m'] <= 1 or
                    not finite(d.get('xy_error_m')) or not 0 < d['xy_error_m'] <= 1):
                raise ValueError('Located detection requires measured footprint and uncertainty')
            clean.append(d)
        previous_sequence = self.last_sequence
        self.last_sequence, self.epoch = sequence, epoch
        used, new_confirmed = set(), []
        for d in clean:
            candidates = [x for x in self.items if x['class'] == d['class'] and x['id'] not in used
                          and math.dist(x['xy_m'], d['xy_m']) <= self.merge_m]
            if candidates:
                item = min(candidates, key=lambda x: math.dist(x['xy_m'], d['xy_m']))
                matched_previous = item['sequence'] == previous_sequence
                item['hits'] = item['hits'] + 1 if (not consecutive or matched_previous) else 1
                # A confirmed footprint never shrinks on one noisy observation.
                if not item['confirmed']:
                    item['xy_m'] = [(a*(item['hits']-1)+b)/item['hits'] for a, b in zip(item['xy_m'], d['xy_m'])]
                item['radius_m'] = max(item['radius_m'], d['radius_m'] + math.dist(item['xy_m'], d['xy_m']))
                item['xy_error_m'] = max(item['xy_error_m'], d['xy_error_m'])
            else:
                if len(self.items) >= self.limit:
                    raise ValueError('Object catalogue full; stop and retain evidence')
                item = {'id': len(self.items)+1, 'class': d['class'], 'xy_m': list(d['xy_m']),
                        'radius_m': d['radius_m'], 'xy_error_m': d['xy_error_m'],
                        'hits': 1, 'confirmed': False, 'first_seen': now}
                self.items.append(item)
            item.update(sequence=sequence, last_seen=now)
            used.add(item['id'])
            if not item['confirmed'] and item['hits'] >= self.confirmations:
                item['confirmed'] = True; new_confirmed.append(item['id'])
        return new_confirmed

    def snapshot(self):
        return deepcopy(self.items)
