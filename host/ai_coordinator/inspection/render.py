"""Render the AI content area on RK. Menus/emergency overlays stay on FPGA."""
from functools import lru_cache
import math
import cv2
import numpy as np
from ..ai_link import font_rows, glyph
from .grid import SIZE, cell

WIDTH, HEIGHT = 816, 420
# Match rtl/ui/ui_render.v layer_color[4] (occupied navigation cells), RGB E87946.
NAV_OBSTACLE_BGR = (0x46, 0x79, 0xE8)
NAMES = {'tape': '胶带', 'screwdriver': '螺丝刀', 'scissors': '剪刀',
         'ad_calcium_milk': 'AD钙奶', 'nutri_express': '营养快线'}
STATES = {'edit': '移动准心选点', 'ready': '确认开始巡检', 'await_start': '等待导航受理',
          'object_recognition':'目标识别',
          'outbound': '去程巡检', 'returning': '最新地图返航', 'no_path': '无路，等待或暂停',
          'await_stop': '停车复核物体', 'await_layer': '提交视觉障碍',
          'await_return_layer': '更新返航障碍', 'paused': '已暂停，确认继续',
          'manual_paused': '手动接管，任务暂停', 'completed': '巡检完成，返回可新建',
          'fault': '控制故障', 'invalidated': '定位重置，任务失效', 'cancelled': '任务已取消'}


@lru_cache(maxsize=1)
def atlas():
    return np.asarray(font_rows(), dtype=np.uint16)


def text(image, value, x, y, colour=(235, 240, 248)):
    for char in str(value):
        if x+16 > image.shape[1] or y+16 > image.shape[0]:
            break
        if x >= 0 and y >= 0:
            bits = (atlas()[glyph(char)][:, None] >> np.arange(15, -1, -1)) & 1
            image[y:y+16, x:x+16][bits.astype(bool)] = colour
        x += 16


def map_pixel(point):
    x, y = cell(point)
    return round((x+.5)*HEIGHT/SIZE), round((SIZE-y-.5)*HEIGHT/SIZE)


def render(view, mission, objects, *, camera=None, detections=(), radar_raw=None,
           cursor=None, scroll=0, calibration_ready=False):
    image = np.full((HEIGHT, WIDTH, 3), (32, 24, 16), np.uint8)
    if view == 'camera':
        if camera is None:
            text(image, '摄像头未连接', 24, 80)
        else:
            source = camera['bgr']
            if source.dtype != np.uint8 or source.ndim != 3 or source.shape[2] != 3:
                raise ValueError('Expected actual BGR camera frame')
            h, w = source.shape[:2]; scale = min(WIDTH/w, (HEIGHT-48)/h)
            size = (max(1, round(w*scale)), max(1, round(h*scale)))
            x0, y0 = (WIDTH-size[0])//2, 36+(HEIGHT-48-size[1])//2
            image[y0:y0+size[1], x0:x0+size[0]] = cv2.resize(source, size)
            for d in list(detections)[:64]:
                if d.get('frame_sequence') != camera['sequence']:
                    continue  # A box from an old image must never follow a new frame.
                box = d.get('bbox', [])
                if len(box) != 4 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in box):
                    continue
                x1, y1, x2, y2 = box
                if not 0 <= x1 < x2 <= w or not 0 <= y1 < y2 <= h:
                    continue
                a, b = (round(x0+x1*scale), round(y0+y1*scale)), (round(x0+x2*scale), round(y0+y2*scale))
                cv2.rectangle(image, a, b, (0, 210, 255), 2)
                text(image, NAMES.get(d.get('class'), d.get('class', '?')), a[0], max(36, a[1]-18))
    elif view == 'map':
        if radar_raw is not None:
            if len(radar_raw) != SIZE*SIZE//8:
                raise ValueError('Only a complete radar map may be displayed')
            occupancy = np.unpackbits(np.frombuffer(radar_raw, np.uint8), bitorder='little').reshape(SIZE, SIZE)
            bitmap = np.zeros((SIZE, SIZE, 3), np.uint8)
            bitmap[:] = (45, 39, 30); bitmap[occupancy.astype(bool)] = NAV_OBSTACLE_BGR
            image[:, :HEIGHT] = cv2.resize(bitmap[::-1], (HEIGHT, HEIGHT), interpolation=cv2.INTER_NEAREST)
        trace = mission.get('trace_m', [])
        if len(trace) > 1:
            cv2.polylines(image, [np.array([map_pixel(p) for p in trace])], False, (230, 160, 40), 2)
        returning=mission.get('return_trace_m',[])
        if len(returning)>1:cv2.polylines(image,[np.array([map_pixel(p) for p in returning])],False,(80,220,80),2)
        planned=mission.get('planned_route',[])
        if len(planned)>1:cv2.polylines(image,[np.array([map_pixel(p) for p in planned])],False,(220,220,80),1)
        pose=mission.get('pose')
        if pose:
            centre=map_pixel([pose['x_m'],pose['y_m']]);yaw=pose['yaw_rad']
            end=(round(centre[0]+15*math.cos(yaw)),round(centre[1]-15*math.sin(yaw)))
            cv2.arrowedLine(image,centre,end,(255,255,255),2,tipLength=.35)
        for obj in objects:
            if obj['confirmed']:
                centre = map_pixel(obj['xy_m'])
                cv2.circle(image, centre, 4, (30, 35, 255), -1)
                radius = max(5, round(obj['xy_error_m']*HEIGHT/12.8))
                cv2.circle(image, centre, radius, (40, 75, 180), 1)
                text(image, obj['id'], centre[0]+6, centre[1]-8, (40, 70, 255))
        if mission.get('goal_m') is not None:
            cv2.drawMarker(image, map_pixel(mission['goal_m']), (0, 220, 255), cv2.MARKER_DIAMOND, 14, 2)
        if mission.get('start_m') is not None:
            cv2.drawMarker(image,map_pixel(mission['start_m']),(80,220,80),cv2.MARKER_SQUARE,12,2)
        if cursor is not None and mission['state'] in ('edit', 'ready'):
            cv2.drawMarker(image, map_pixel(cursor), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
        text(image, '橙格：导航障碍', 440, 40)
        text(image, '红点：已定位目标', 440, 64)
        text(image, '黄点：巡检终点', 440, 92)
        text(image, '蓝线：实际去程轨迹', 440, 120)
        text(image, '绿点：返航起点，可换路', 440, 144)
        text(image, '坐标：起始车头向前为X', 440, 172)
        text(image, '地图范围：12.8米', 440, 200)
    elif view == 'list':
        confirmed = [o for o in objects if o['confirmed']]
        text(image, '编号    目标            X/米       Y/米', 16, 48)
        for row, obj in enumerate(confirmed[max(0, scroll):max(0, scroll)+12]):
            x, y = obj['xy_m']
            line = '%03d  %-6s  %+.2f   %+.2f' % (obj['id'], NAMES.get(obj['class'], obj['class']), x, y)
            text(image, line, 16, 80+row*24)
        if not confirmed:
            text(image, '尚无完成定位的目标', 24, 100)
    else:
        raise ValueError('Unknown inspection view')
    cv2.rectangle(image, (0, 0), (WIDTH-1, 31), (32, 24, 16), -1)
    caption = {'recording':'录音中，上拨结束','recognizing':'正在识别语音'}.get(mission.get('recording_phase'),STATES.get(mission['state'], mission['state']))
    text(image, caption, 12, 8)
    if mission.get('instruction'):
        text(image,str(mission['instruction'])[:48],12,HEIGHT-18,(0,220,255))
    if not calibration_ready:
        text(image, '未标定：仅观察，不授权运动', 352, 8, (20, 170, 255))
    return image


def render_recognition(camera,detections,*,instruction='',recording_phase=None):
    # Reuse the same annotated pixels; standalone recognition has no metric
    # localization requirement and must not show inspection calibration text.
    return render('camera',dict(state='object_recognition',instruction=instruction,recording_phase=recording_phase),
                  [],camera=camera,detections=detections,calibration_ready=True)


def rgb565(image):
    if image.shape != (HEIGHT, WIDTH, 3) or image.dtype != np.uint8:
        raise ValueError('Only the fixed AI content rectangle may be transmitted')
    b, g, r = [image[:, :, k].astype(np.uint16) for k in range(3)]
    return (((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)).astype('<u2').tobytes()
