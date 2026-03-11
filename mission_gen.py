import json
import math
import zipfile
import os
from datetime import datetime

# ==========================================
# MASTER CONFIGURATION (Inputs in Feet/FPS)
# ==========================================
CONFIG = {
    "mission_name": "ISLERS_Native_Flight",
    "geojson_path": "missions/latest_path.geojson",
    
    # Global Config
    "safe_takeoff_ft": 60.0,      
    "trans_speed_fps": 32.0,      
    
    # Waypoint Settings
    "alt_ft": 50.0,               
    "gimbal_pitch": -60,          
    "side": "right",              
    
    # Photo Trigger
    "photo_start_wp": 0,          # 0 starts at the very beginning of the path
    "trigger_type": "distance",   
    "interval_ft": 9.0,           
    "interval_sec": 2.0,          
    
    # Mission Speed
    "auto_calc_speed": True,
    "target_gap_ft": 26.2,        
    "manual_fps": 13.5            
}

FT_TO_M = 0.3048

def get_haversine_dist(p1, p2):
    R = 6371000 
    lat1, lon1, lat2, lon2 = map(math.radians, [p1[0], p1[1], p2[0], p2[1]])
    dlat, dlon = lat1 - lat2, lon1 - lon2
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def get_bearing(p1, p2):
    lat1, lon1, lat2, lon2 = map(math.radians, [p1[0], p1[1], p2[0], p2[1]])
    d_lon = lon2 - lon1
    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def generate_native_kmz_contents(coords, cfg):
    ms_ts = int(datetime.now().timestamp() * 1000)
    
    alt_m = cfg["alt_ft"] * FT_TO_M
    safe_m = cfg["safe_takeoff_ft"] * FT_TO_M
    trans_m = cfg["trans_speed_fps"] * FT_TO_M
    
    speed_m = cfg.get("speed_m")
    if speed_m is None:
        speed_m = min(max((cfg["target_gap_ft"] * FT_TO_M) / cfg["interval_sec"], 1.0), 10.0) if (cfg["auto_calc_speed"] and cfg["trigger_type"] == "time") else (cfg["manual_fps"] * FT_TO_M)

    total_dist_m = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(len(coords)-1))
    total_duration = total_dist_m / speed_m if speed_m > 0 else 0
    pitch_val = cfg['pitch'] if 'pitch' in cfg else cfg.get('gimbal_pitch', -60)

    # =========================================================
    # 1. BUILD TEMPLATE.KML (For the Controller UI)
    # =========================================================
    template_placemarks = ""
    for i, p in enumerate(coords):
        # YAW MATH: Convert 0-360 to DJI's -180 to 180 format
        ref_bearing = get_bearing(coords[i], coords[i+1]) if i < len(coords)-1 else get_bearing(coords[i-1], coords[i])
        yaw = (ref_bearing + 90) % 360 if cfg['side'] == "right" else (ref_bearing - 90) % 360
        if yaw > 180:
            yaw -= 360

        # Inject Photo UI Trigger into the Template
        template_photo_action = ""
        start_wp = cfg.get("photo_start_wp", 0)
        if i == start_wp:
            trigger_tag = "multipleDistance" if cfg["trigger_type"] == "distance" else "multipleTiming"
            interval_val = (cfg['interval_ft'] * FT_TO_M) if cfg["trigger_type"] == "distance" else cfg['interval_sec']
            template_photo_action = f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>0</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{start_wp}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{len(coords)-1}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>{trigger_tag}</wpml:actionTriggerType><wpml:actionTriggerParam>{interval_val:.2f}</wpml:actionTriggerParam></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
              <wpml:useGlobalPayloadLensIndex>1</wpml:useGlobalPayloadLensIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""

        template_placemarks += f"""
      <Placemark>
        <Point><coordinates>{p[1]:.8f},{p[0]:.8f}</coordinates></Point>
        <wpml:index>{i}</wpml:index>
        <wpml:ellipsoidHeight>{alt_m:.1f}</wpml:ellipsoidHeight>
        <wpml:height>{alt_m:.1f}</wpml:height>
        <wpml:useGlobalHeight>1</wpml:useGlobalHeight>
        <wpml:useGlobalSpeed>1</wpml:useGlobalSpeed>
        <wpml:useGlobalTurnParam>1</wpml:useGlobalTurnParam>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>smoothTransition</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>{int(yaw)}</wpml:waypointHeadingAngle>
          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>
          <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>
        </wpml:waypointHeadingParam>
        <wpml:gimbalPitchAngle>{pitch_val}</wpml:gimbalPitchAngle>
        <wpml:isRisky>0</wpml:isRisky>
        {template_photo_action}
      </Placemark>"""

    template_kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
  <Document>
    <wpml:createTime>{ms_ts}</wpml:createTime>
    <wpml:updateTime>{ms_ts}</wpml:updateTime>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:takeOffSecurityHeight>{safe_m:.1f}</wpml:takeOffSecurityHeight>
      <wpml:globalTransitionalSpeed>{trans_m:.1f}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo><wpml:droneEnumValue>77</wpml:droneEnumValue><wpml:droneSubEnumValue>2</wpml:droneSubEnumValue></wpml:droneInfo>
      <wpml:payloadInfo><wpml:payloadEnumValue>68</wpml:payloadEnumValue><wpml:payloadSubEnumValue>0</wpml:payloadSubEnumValue><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:payloadInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateType>waypoint</wpml:templateType>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineCoordinateSysParam>
        <wpml:coordinateMode>WGS84</wpml:coordinateMode>
        <wpml:heightMode>relativeToStartPoint</wpml:heightMode>
        <wpml:positioningType>GPS</wpml:positioningType>
      </wpml:waylineCoordinateSysParam>
      <wpml:autoFlightSpeed>{speed_m:.1f}</wpml:autoFlightSpeed>
      <wpml:globalHeight>{alt_m:.1f}</wpml:globalHeight>
      <wpml:caliFlightEnable>0</wpml:caliFlightEnable>
      <wpml:gimbalPitchMode>usePointSetting</wpml:gimbalPitchMode>
      <wpml:globalWaypointHeadingParam>
        <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
        <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
        <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>
      </wpml:globalWaypointHeadingParam>
      <wpml:globalWaypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:globalWaypointTurnMode>
      <wpml:globalUseStraightLine>1</wpml:globalUseStraightLine>
      {template_placemarks}
    </Folder>
  </Document>
</kml>"""

    # =========================================================
    # 2. BUILD WAYLINES.WPML (For Drone Execution)
    # =========================================================
    waylines_placemarks = ""
    global_action_id = 0  

    for i, p in enumerate(coords):
        ref_bearing = get_bearing(coords[i], coords[i+1]) if i < len(coords)-1 else get_bearing(coords[i-1], coords[i])
        yaw = (ref_bearing + 90) % 360 if cfg['side'] == "right" else (ref_bearing - 90) % 360
        if yaw > 180:
            yaw -= 360

        action_group = ""
        
        # WAYPOINT 0 GIMBAL SNAP
        if i == 0:
            action_group += f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>{global_action_id}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>0</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>reachPoint</wpml:actionTriggerType></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:gimbalHeadingYawBase>aircraft</wpml:gimbalHeadingYawBase>
              <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>
              <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>
              <wpml:gimbalPitchRotateAngle>{pitch_val}</wpml:gimbalPitchRotateAngle>
              <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>
              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
              <wpml:gimbalYawRotateEnable>0</wpml:gimbalYawRotateEnable>
              <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
              <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>
              <wpml:gimbalRotateTime>10</wpml:gimbalRotateTime>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""
            global_action_id += 1

        # PHOTO TRIGGER (Matches Template)
        if i == start_wp:
            trigger_tag = "multipleDistance" if cfg["trigger_type"] == "distance" else "multipleTiming"
            interval_val = (cfg['interval_ft'] * FT_TO_M) if cfg["trigger_type"] == "distance" else cfg['interval_sec']
            action_group += f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>{global_action_id}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{start_wp}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{len(coords)-1}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>{trigger_tag}</wpml:actionTriggerType><wpml:actionTriggerParam>{interval_val:.2f}</wpml:actionTriggerParam></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
              <wpml:useGlobalPayloadLensIndex>1</wpml:useGlobalPayloadLensIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""
            global_action_id += 1

        # GIMBAL HOLD BETWEEN POINTS
        if i < len(coords) - 1:
            action_group += f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>{global_action_id}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{i}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{i+1}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>betweenAdjacentPoints</wpml:actionTriggerType></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>gimbalEvenlyRotate</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:gimbalPitchRotateAngle>{pitch_val}</wpml:gimbalPitchRotateAngle>
              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""
            global_action_id += 1

        waylines_placemarks += f"""
      <Placemark>
        <Point><coordinates>{p[1]:.8f},{p[0]:.8f}</coordinates></Point>
        <wpml:index>{i}</wpml:index>
        <wpml:executeHeight>{alt_m:.1f}</wpml:executeHeight>
        <wpml:waypointSpeed>{speed_m:.2f}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>smoothTransition</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>{int(yaw)}</wpml:waypointHeadingAngle>
          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
          <wpml:waypointHeadingAngleEnable>1</wpml:waypointHeadingAngleEnable>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>
          <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>
        </wpml:waypointHeadingParam>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
        <wpml:useStraightLine>1</wpml:useStraightLine>
        <wpml:waypointGimbalHeadingParam>
          <wpml:waypointGimbalPitchAngle>{pitch_val}</wpml:waypointGimbalPitchAngle>
          <wpml:waypointGimbalYawAngle>0</wpml:waypointGimbalYawAngle>
        </wpml:waypointGimbalHeadingParam>
        <wpml:isRisky>0</wpml:isRisky>
        <wpml:waypointWorkType>0</wpml:waypointWorkType>
        {action_group}
      </Placemark>"""

    waylines_wpml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
  <Document>
    <wpml:createTime>{ms_ts}</wpml:createTime>
    <wpml:updateTime>{ms_ts}</wpml:updateTime>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:takeOffSecurityHeight>{safe_m:.1f}</wpml:takeOffSecurityHeight>
      <wpml:globalTransitionalSpeed>{trans_m:.1f}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo><wpml:droneEnumValue>77</wpml:droneEnumValue><wpml:droneSubEnumValue>2</wpml:droneSubEnumValue></wpml:droneInfo>
      <wpml:waylineAvoidLimitAreaMode>0</wpml:waylineAvoidLimitAreaMode>
      <wpml:payloadInfo><wpml:payloadEnumValue>68</wpml:payloadEnumValue><wpml:payloadSubEnumValue>0</wpml:payloadSubEnumValue><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:payloadInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:distance>{total_dist_m:.2f}</wpml:distance>
      <wpml:duration>{total_duration:.2f}</wpml:duration>
      <wpml:autoFlightSpeed>{speed_m:.1f}</wpml:autoFlightSpeed>
      {waylines_placemarks}
    </Folder>
  </Document>
</kml>"""

    return template_kml, waylines_wpml


def run_gen():
    if not os.path.exists(CONFIG["geojson_path"]):
        print(f"Error: Could not find {CONFIG['geojson_path']}")
        return
        
    with open(CONFIG["geojson_path"], 'r', encoding='utf-8') as f:
        data = json.load(f)
        
    coords = [(c[1], c[0]) for c in data['geometry']['coordinates']]
    
    template_kml, waylines_wpml = generate_native_kmz_contents(coords, CONFIG)
    
    os.makedirs("missions", exist_ok=True)
    file_path = f"missions/{CONFIG['mission_name']}.kmz"
    
    with zipfile.ZipFile(file_path, 'w') as kmz:
        kmz.writestr("wpmz/waylines.wpml", waylines_wpml)
        kmz.writestr("wpmz/template.kml", template_kml)
        
    print(f"Success! Native mission generated: {file_path}")

if __name__ == "__main__":
    run_gen()