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
STATES = {'edit': '移动准心，SWA打开选点菜单', 'ready': '确认路线后开始巡检', 'await_start': '等待导航受理',
          'object_recognition':'目标识别',
          'outbound': '去程巡检', 'returning': '最新地图返航', 'no_path': '无路，等待或暂停',
          'await_stop': '停车复核物体', 'await_layer': '提交视觉障碍',
          'observe_stop':'识别目标，等待停车', 'observe_wait':'停车后等待0.5秒', 'observing':'停稳后定位（3帧）',
          'await_return_layer': '更新返航障碍', 'paused': '已暂停，确认继续',
          'manual_paused': '手动接管，任务暂停', 'completed': '巡检完成，SWA新建，返回退出',
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


def list_entries(mission, objects):
    points=mission.get('waypoints_m',[]);completed=mission.get('waypoints_completed',0)
    index=mission.get('waypoint_index',0)
    entries=['%-6s%-7s%7s  %7s'%('编号','状态','X/米','Y/米')]
    for i,(x,y) in enumerate(points):
        status='已完成' if i<completed else '当前点' if i==index and mission['state']!='edit' else '待执行'
        entries.append('%-6s%-7s%+7.2f  %+7.2f'%('P%d'%(i+1),status,x,y))
    if not points:entries.append('尚未添加路径点')
    entries.append('%-6s%-9s%7s  %7s'%('编号','目标名称','X/米','Y/米'))
    for obj in objects:
        if obj['confirmed']:
            x,y=obj['xy_m']
            entries.append('%-6s%-9s%+7.2f  %+7.2f'%('%03d'%obj['id'],NAMES.get(obj['class'],obj['class']),x,y))
    if not any(o['confirmed'] for o in objects):entries.append('尚无完成定位的目标')
    return entries


def dashed_line(image, a, b):
    length=math.dist(a,b)
    if not length:return
    for offset in range(0,math.ceil(length),10):
        ends=[tuple(round(a[i]+(b[i]-a[i])*d/length) for i in (0,1)) for d in (offset,min(offset+5,length))]
        cv2.line(image,*ends,(145,130,95),1)


def waypoint_panel(image,mission):
    points=mission.get('waypoints_m',[]);completed=mission.get('waypoints_completed',0)
    index=mission.get('waypoint_index',0);state=mission['state']
    text(image,'路径点：%d/%d已完成'%(completed,len(points)),440,40)
    text(image,'坐标X向前、Y向左，单位米',440,64)
    notice=mission.get('notice','')
    for row in range(2):text(image,notice[row*22:(row+1)*22],440,88+row*20,(0,220,255))
    selected=mission.get('edit_action')
    if selected is not None:
        for i,label in enumerate(('添加路径点','撤销末点','清空路径点','开始巡检')):
            y=144+i*40
            if i==selected:cv2.rectangle(image,(436,y-7),(WIDTH-8,y+25),(170,75,0),-1)
            text(image,label,456,y,(0,240,255) if i==selected else (235,240,248))
        text(image,'摇杆上下选择，SWA执行',440,328)
        text(image,'返回或CH8切页关闭菜单',440,352)
        return
    if state=='ready':
        text(image,'共%d点，完成后自动返航'%len(points),440,144)
        text(image,'SWA开始；返回继续编辑',440,168)
    else:
        text(image,'橙格：障碍；红点：目标',440,144)
        text(image,'浅蓝待执行、黄当前、绿完成',440,168)
    text(image,'编号       X/米      Y/米',440,204)
    first=max(0,len(points)-6) if state in ('edit','ready') else max(0,min(index-1,len(points)-6))
    for row,i in enumerate(range(first,min(first+6,len(points)))):
        x,y=points[i];colour=(80,220,80) if i<completed else (0,220,255) if i==index and state!='edit' else (255,210,150)
        text(image,'P%-3d  %+.2f  %+.2f'%(i+1,x,y),440,230+row*24,colour)
    text(image,'SWA打开菜单' if state=='edit' else 'CH8切页；SWA暂停或继续',440,392)


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
        points=mission.get('waypoints_m',[])
        ordered=([mission['start_m']] if mission.get('start_m') is not None else [])+points
        for a,b in zip(ordered,ordered[1:]):dashed_line(image,map_pixel(a),map_pixel(b))
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
        for i,point in enumerate(points):
            colour=(80,220,80) if i<mission.get('waypoints_completed',0) else (0,220,255) if i==mission.get('waypoint_index',0) and mission['state']!='edit' else (255,210,150)
            centre=map_pixel(point)
            cv2.drawMarker(image,centre,colour,cv2.MARKER_DIAMOND,14,2)
            text(image,'P%d'%(i+1),centre[0]+8,centre[1]-18,colour)
        if mission.get('start_m') is not None:
            cv2.drawMarker(image,map_pixel(mission['start_m']),(80,220,80),cv2.MARKER_SQUARE,12,2)
        if cursor is not None and mission['state'] in ('edit', 'ready'):
            cv2.drawMarker(image, map_pixel(cursor), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
        # Markers at the map's right edge must not overwrite the sidebar.
        image[:,HEIGHT:]=(32,24,16)
        waypoint_panel(image,mission)
    elif view == 'list':
        text(image,'巡检路径点与目标记录（摇杆上下滚动）',16,48)
        entries=list_entries(mission,objects);offset=max(0,min(scroll,len(entries)-12))
        for row,line in enumerate(entries[offset:offset+12]):text(image,line,16,80+row*24)
    else:
        raise ValueError('Unknown inspection view')
    cv2.rectangle(image, (0, 0), (WIDTH-1, 31), (32, 24, 16), -1)
    caption = {'recording':'录音中，上拨结束','recognizing':'正在识别语音'}.get(mission.get('recording_phase'),STATES.get(mission['state'], mission['state']))
    if mission.get('waypoint_count'):
        if mission['state'] in ('edit','ready'):progress='待开始%d点'%mission['waypoint_count']
        elif mission['state']=='completed':progress='已完成%d点'%mission['waypoint_count']
        else:progress='返航' if mission.get('phase')=='returning' else '去程%d/%d'%(min(mission['waypoint_index']+1,mission['waypoint_count']),mission['waypoint_count'])
        caption+=' | '+progress
    if mission.get('manual'):caption='手动接管，返回光标后确认继续'
    text(image, caption if calibration_ready else caption[:20], 12, 8)
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
