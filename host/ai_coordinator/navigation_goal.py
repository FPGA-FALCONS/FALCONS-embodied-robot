"""Candidate parking goal in the current FPGA navigation_origin frame.

Grid contract verified against upstream ec759b1: 256 cells, 50 mm/cell,
[-6400,6400) mm. This is a proposal: traversability and arrival stay on FPGA.
"""
import math
from .ground_target import base_to_map
from .schema import number


def cell(mm):
    if not number(mm) or not -6400<=mm<6400:raise ValueError('Outside FPGA navigation map')
    return math.floor((mm+6400)/50)


def parking_proposal(point_base_m,pose_m_rad,standoff_m,*,frame_id,localization_epoch,
                     current_epoch,captured_at_s,pose_at_s,now_s):
    if not all(isinstance(s,str) and s for s in (frame_id,localization_epoch,current_epoch)):raise ValueError('Source frame and localization epoch required')
    if localization_epoch!=current_epoch:raise ValueError('Localization reset invalidated visual goal')
    if not all(number(x) for x in (standoff_m,captured_at_s,pose_at_s,now_s)) or standoff_m<=0:raise ValueError('Invalid metric proposal inputs')
    if not 0<=now_s-captured_at_s<=1 or abs(captured_at_s-pose_at_s)>.02:raise ValueError('Stale image or mismatched capture pose')
    target=base_to_map(point_base_m,pose_m_rad)
    dx=target[0]-pose_m_rad[0];dy=target[1]-pose_m_rad[1];distance=math.hypot(dx,dy)
    if distance<=standoff_m:raise ValueError('Already within stand-off; evaluate stationary state instead')
    x_mm=(target[0]-standoff_m*dx/distance)*1000
    y_mm=(target[1]-standoff_m*dy/distance)*1000
    return {'schema_version':1,'type':'ai_navigation_goal_proposal','frame':'navigation_origin',
            'source_frame_id':frame_id,'localization_epoch':localization_epoch,
            'goal_x_mm':x_mm,'goal_y_mm':y_mm,'goal_x_cell':cell(x_mm),'goal_y_cell':cell(y_mm),
            'goal_valid':False,'motion_authorized':False,'reason':'parking_point_requires_planner_clearance_check',
            'assumptions':['measured_camera_calibration','verified_ground_contact','pose_at_capture',
                           'vehicle_clearance_and_braking_margin_not_yet_verified']}
