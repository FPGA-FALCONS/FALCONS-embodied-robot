"""Receipt-driven inspection scheduler. It never writes motors or serial ports.

Commands are proposals consumed by the single transport owner. Every physical
receipt is bound to a session, task, localization epoch and command. Manual and
timeout pauses discard in-flight progress; going back to cursor mode is not a
resume command. Internal obstacle pauses may resume after a committed layer.
"""
from collections import deque
from copy import deepcopy
import math
import uuid
import zlib
from .calibration import finite
from .grid import cell, PLANE_BYTES

REQUIRED = ('calibration', 'timing', 'camera', 'fpga', 'map', 'layer', 'speed_mapping', 'vehicle')
ACTIVE = {'await_start', 'outbound', 'returning', 'no_path', 'await_stop', 'await_layer', 'await_return_layer'}


class InspectionMission:
    def __init__(self, *, session_id=None, trace_step_m=.15, trace_limit=512):
        self.session_id = session_id or (uuid.uuid4().int & 0xffffffff) or 1
        self.trace_step_m, self.trace_limit = trace_step_m, trace_limit
        self.task_id = 0; self.command_id = 0; self.layer_version = 0
        self.state = 'edit'; self.phase = 'outbound'; self.view = 'map'
        self.goal = None; self.pose = None; self.epoch = None
        self.start_point = None
        self.trace = []; self.return_points = []; self.return_index = 0
        self.actions = deque(); self.expected = None; self.pending_layer = None
        self.reason = None; self.manual = False; self.last_status = None
        self.trace_yaw = None

    def _command(self, kind, now, **fields):
        self.command_id += 1
        command = {'kind': kind, 'session_id': self.session_id, 'task_id': self.task_id,
                   'epoch': self.epoch, 'command_id': self.command_id, 'at_s': now, **fields}
        self.expected = command
        self.actions.append(command)
        return command

    def drain(self):
        result = list(self.actions); self.actions.clear(); return result

    def set_view(self, index):
        if type(index) is not int or not 0 <= index <= 2:
            raise ValueError('Inspection view must be camera/map/list')
        self.view = ('camera', 'map', 'list')[index]

    def set_pose(self, pose, now):
        if (not finite(now) or type(pose.get('epoch')) is not int or not 0 < pose['epoch'] <= 0xffffffff or
                not all(finite(pose.get(k)) for k in ('x_m', 'y_m', 'yaw_rad'))):
            raise ValueError('Invalid actual pose snapshot')
        cell((pose['x_m'], pose['y_m']))
        if self.epoch not in (None, pose['epoch']):
            self.cancel(now, 'localization_reset')
            self.state = 'invalidated'
        self.epoch = pose['epoch']; self.pose = {**pose, 'at_s': now}
        if self.state == 'outbound':
            try:
                self._trace()
            except ValueError:
                self.pause(now, 'trajectory_full')

    def _trace(self, force=False):
        if self.pose is None:
            return
        point = [self.pose['x_m'], self.pose['y_m']]
        if self.trace:
            distance = math.dist(point, self.trace[-1])
            angle = abs((self.pose['yaw_rad']-self.trace_yaw+math.pi) % (2*math.pi)-math.pi)
            if distance < .001 or (not force and distance < self.trace_step_m and angle < math.pi/12):
                return
        if len(self.trace) >= self.trace_limit:
            raise ValueError('Actual trajectory capacity reached; pause required')
        self.trace.append(point)
        self.trace_yaw = self.pose['yaw_rad']

    def select_goal(self, xy):
        if self.state not in ('edit', 'ready'):
            raise ValueError('Goal editing is disabled during a task or on completion')
        cell(xy)
        self.goal = list(xy); self.state = 'ready'; self.reason = None

    def _problems(self, ready, now):
        errors = [key for key in REQUIRED if ready.get(key) is not True]
        if self.manual:
            errors.append('manual')
        if self.pose is None or not finite(now) or not 0 <= now-self.pose['at_s'] <= .5:
            errors.append('pose_stale')
        return errors

    def start(self, ready, now):
        if self.state != 'ready' or self.goal is None:
            raise ValueError('A new confirmed goal is required')
        problems = self._problems(ready, now)
        if problems:
            self.reason = 'not_ready:' + ','.join(problems); return False
        self.task_id += 1; self.phase = 'outbound'; self.trace = []
        self.start_point = [self.pose['x_m'],self.pose['y_m']]
        self.return_points = []; self.return_index = 0; self.pending_layer = None
        self.reason = None; self._trace(force=True); self.last_status = now
        self._goto(now)
        return True

    def _goto(self, now):
        goal = self.goal if self.phase == 'outbound' else self.return_points[self.return_index]
        self.state = 'await_start'
        self._command('goto', now, goal_mm=[round(v*1000) for v in goal],
                      layer_version=self.layer_version, speed_code=30, phase=self.phase)

    def pause(self, now, reason='operator_pause', manual=False):
        self.manual = manual or self.manual
        if self.state in ACTIVE or self.state in ('paused', 'manual_paused', 'fault'):
            self.actions.clear()  # No old queued goto survives a stop request.
            self._command('pause', now, reason=reason)
            self.state = 'manual_paused' if self.manual else 'paused'
            self.reason = reason

    def set_manual(self, enabled, now):
        if enabled:
            self.pause(now, 'manual_takeover', manual=True)
        else:
            self.manual = False
            if self.state == 'manual_paused':
                self.state = 'paused'  # No _goto and no automatic resumption.

    def resume(self, ready, now):
        if self.state not in ('paused', 'fault'):
            return False
        problems = self._problems(ready, now)
        if problems:
            self.reason = 'not_ready:' + ','.join(problems); return False
        self.reason = None; self.last_status = now
        if self.pending_layer is not None:
            self.state = 'await_stop'; self._command('pause', now, reason='layer_update')
        else:
            self._goto(now)
        return True

    def cancel(self, now, reason='cancelled'):
        self.actions.clear()
        if self.state in ACTIVE or self.state in ('paused', 'manual_paused', 'fault'):
            self._command('cancel', now, reason=reason)
        self.state = 'cancelled'; self.reason = reason; self.pending_layer = None

    def new_task(self):
        if self.state not in ('completed', 'cancelled', 'invalidated', 'edit'):
            raise ValueError('Stop or cancel the current task first')
        self.actions.clear(); self.expected = None; self.pending_layer = None
        self.state = 'edit'; self.phase = 'outbound'; self.goal = None
        self.trace = []; self.return_points = []; self.reason = None
        self.start_point = None

    def submit_layer(self, layer, now):
        raw, cost = layer.get('raw'), layer.get('cost')
        if (not isinstance(raw, bytes) or not isinstance(cost, bytes) or
                len(raw) != PLANE_BYTES or len(cost) != PLANE_BYTES or
                zlib.crc32(raw+cost) & 0xffffffff != layer.get('crc32') or
                any(a & ~b for a, b in zip(raw, cost))):
            raise ValueError('Layer must be complete, CRC-valid and cost must contain raw')
        if self.state not in ('outbound', 'returning', 'no_path', 'await_return_layer'):
            raise ValueError('Layer update requires a running task or stopped return preparation')
        if self.layer_version == 65535:
            raise ValueError('Layer version exhausted; cancel and re-establish session')
        self.pending_layer = {**layer, 'version': self.layer_version+1}
        if self.state == 'await_return_layer':
            self._commit_layer(now)  # Arrival receipt already proved stopped.
        else:
            self.state = 'await_stop'; self.actions.clear()
            self._command('pause', now, reason='layer_update')

    def _commit_layer(self, now):
        self.state = 'await_layer'
        self._command('commit_layer', now, layer_version=self.pending_layer['version'],
                      crc32=self.pending_layer['crc32'])

    def on_status(self, event, now):
        if not self.expected or any(event.get(k) != self.expected[k] for k in
                                    ('session_id', 'task_id', 'epoch', 'command_id')):
            return False
        if not finite(now):
            return False
        kind = event.get('kind'); self.last_status = now
        if kind == 'fault' and self.state in ACTIVE:
            self.pause(now, event.get('reason', 'controller_fault')); self.state = 'fault'
        elif kind == 'accepted' and self.state == 'await_start':
            self.state = self.phase
        elif kind == 'no_path' and self.state in ('outbound', 'returning', 'await_start'):
            self.state = 'no_path'
        elif kind == 'running' and self.state == 'no_path':
            self.state = self.phase
        elif kind == 'stopped' and self.state == 'await_stop' and event.get('stopped') is True:
            self._commit_layer(now)
        elif kind == 'layer_committed' and self.state == 'await_layer' and event.get('layer_version') == self.pending_layer['version']:
            self.layer_version = self.pending_layer['version']; self.pending_layer = None
            self._goto(now)
        elif kind == 'arrived' and self.state in ('outbound', 'returning', 'no_path') and event.get('stopped') is True:
            if self.pose is None or not 0 <= now-self.pose['at_s'] <= .5:
                self.pause(now, 'arrival_pose_stale'); return True
            if self.phase == 'outbound':
                self._trace(force=True)
                # The outbound trace is evidence, not a return corridor. The
                # existing FPGA planner may reroute on the latest merged map.
                # Freeze the actual task start rather than the editor's pose
                # or the first planned waypoint. Manual detours do not move it.
                self.return_points = [deepcopy(self.start_point)]
                self.return_index = 0
                self.phase = 'returning'; self.state = 'await_return_layer'
                self._command('prepare_return_layer', now, return_goal_m=deepcopy(self.start_point),
                              return_policy='latest_map',trace=deepcopy(self.trace))
            else:
                self.return_index += 1
                if self.return_index == len(self.return_points):
                    self.state = 'completed'; self.expected = None
                else:
                    self._goto(now)
        else:
            return False
        return True

    def tick(self, now):
        if self.state in ACTIVE and (self.last_status is None or now-self.last_status > .5 or
                                    self.pose is None or now-self.pose['at_s'] > .5):
            self.pause(now, 'control_or_pose_timeout')

    def snapshot(self):
        return {'session_id': self.session_id, 'task_id': self.task_id, 'epoch': self.epoch,
                'state': self.state, 'phase': self.phase, 'view': self.view, 'reason': self.reason,
                'goal_m': deepcopy(self.goal), 'trace_m': deepcopy(self.trace),
                'start_m':deepcopy(self.start_point),'return_policy':'latest_map',
                'return_index': self.return_index, 'return_count': len(self.return_points),
                'layer_version': self.layer_version, 'manual': self.manual}
