import json
import math
import zipfile
import os
from datetime import datetime

# ==========================================
# CONFIGURATION - INPUT IN FEET / FPS
# ==========================================
CONFIG = {
    "altitude_ft": 90.0,         # ~70 meters
    "gimbal_pitch": -60,
    "side": "right",
    "geojson_path": "missions/path_20260303_152823.geojson",
    
    # TRIGGER SETTINGS
    "trigger_type": "distance",   # 'time' or 'distance'
    "interval_ft": 16.4,          # ~5 meters (for 'distance')
    "interval_sec": 2.0,          # Seconds (for 'time')
    
    # SPEED SETTINGS
    "auto_calc_speed": True,
    "manual_speed_fps": 13.5,     # ~4.1 m/s
    "target_dist_ft": 26.2,       # ~8 meters (Desired gap between shots)

    # NATIVE CODES (Mavic 3)
    "drone_enum": "77",           
    "payload_enum": "68"
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

def generate_native_wpml(coords, cfg):
    ms_ts = int(datetime.now().timestamp() * 1000)
    alt_m = cfg["altitude_ft"] * FT_TO_M
    
    if cfg["auto_calc_speed"] and cfg["trigger_type"] == "time":
        target_m = cfg["target_dist_ft"] * FT_TO_M
        speed_m = min(max(target_m / cfg["interval_sec"], 1.0), 10.0)
    else:
        speed_m = cfg["manual_speed_fps"] * FT_TO_M

    total_dist_m = sum(get_haversine_dist(coords[i], coords[i+1]) for i in range(len(coords)-1))
    total_duration = total_dist_m / speed_m

    waypoints_xml = ""
    for i, p in enumerate(coords):
        ref_bearing = get_bearing(coords[i], coords[i+1]) if i < len(coords)-1 else get_bearing(coords[i-1], coords[i])
        yaw = (ref_bearing + 90) % 360 if cfg['side'] == "right" else (ref_bearing - 90) % 360

        action_group = ""
        if i == 0:
            trigger_type = "multipleDistance" if cfg["trigger_type"] == "distance" else "multipleTiming"
            interval_val = (cfg['interval_ft'] * FT_TO_M) if cfg["trigger_type"] == "distance" else cfg['interval_sec']
            
            action_group = f"""
        <wpml:actionGroup>
          <wpml:actionGroupId>0</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{len(coords)-1}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger>
            <wpml:actionTriggerType>{trigger_type}</wpml:actionTriggerType>
            <wpml:actionTriggerParam>{interval_val:.3f}</wpml:actionTriggerParam>
          </wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>"""

        waypoints_xml += f"""
      <Placemark>
        <Point><coordinates>{p[1]},{p[0]},{alt_m:.3f}</coordinates></Point>
        <wpml:index>{i}</wpml:index>
        <wpml:executeHeight>{alt_m:.3f}</wpml:executeHeight>
        <wpml:waypointSpeed>{speed_m:.2f}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>fixed</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>{yaw:.1f}</wpml:waypointHeadingAngle>
          <wpml:waypointHeadingAngleEnable>1</wpml:waypointHeadingAngleEnable>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>
        </wpml:waypointHeadingParam>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
        <wpml:useStraightLine>1</wpml:useStraightLine>
        <wpml:isRisky>0</wpml:isRisky>
        <wpml:waypointWorkType>0</wpml:waypointWorkType>
        {action_group}
      </Placemark>"""

    return f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
  <Document>
    <wpml:createTime>{ms_ts}</wpml:createTime>
    <wpml:updateTime>{ms_ts}</wpml:updateTime>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:takeOffSecurityHeight>20</wpml:takeOffSecurityHeight>
      <wpml:globalTransitionalSpeed>15</wpml:globalTransitionalSpeed>
      <wpml:droneInfo><wpml:droneEnumValue>{cfg['drone_enum']}</wpml:droneEnumValue><wpml:droneSubEnumValue>0</wpml:droneSubEnumValue></wpml:droneInfo>
      <wpml:payloadInfo><wpml:payloadEnumValue>{cfg['payload_enum']}</wpml:payloadEnumValue><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:payloadInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateType>waypoint</wpml:templateType>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:distance>{total_dist_m:.2f}</wpml:distance>
      <wpml:duration>{total_duration:.2f}</wpml:duration>
      <wpml:autoFlightSpeed>{speed_m:.2f}</wpml:autoFlightSpeed>
      <wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>
      <wpml:startActionGroup>
        <wpml:action>
          <wpml:actionId>0</wpml:actionId>
          <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>
            <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>
            <wpml:gimbalPitchRotateAngle>{cfg['gimbal_pitch']}</wpml:gimbalPitchRotateAngle>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>
        <wpml:action><wpml:actionId>1</wpml:actionId><wpml:actionActuatorFunc>setFocusType</wpml:actionActuatorFunc><wpml:actionActuatorFuncParam><wpml:cameraFocusType>manual</wpml:cameraFocusType><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:actionActuatorFuncParam></wpml:action>
        <wpml:action><wpml:actionId>2</wpml:actionId><wpml:actionActuatorFunc>focus</wpml:actionActuatorFunc><wpml:actionActuatorFuncParam><wpml:isInfiniteFocus>1</wpml:isInfiniteFocus><wpml:payloadPositionIndex>0</wpml:payloadPositionIndex></wpml:actionActuatorFuncParam></wpml:action>
      </wpml:startActionGroup>
      {waypoints_xml}
    </Folder>
  </Document>
</kml>"""

def create_mission():
    # FIXED: Added encoding='utf-8' to handle the UnicodeDecodeError
    if not os.path.exists(CONFIG['geojson_path']):
        print(f"Error: {CONFIG['geojson_path']} not found.")
        return

    with open(CONFIG['geojson_path'], 'r', encoding='utf-8') as f:
        data = json.load(f)
    coords = [(c[1], c[0]) for c in data['geometry']['coordinates']]
    
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    kmz_filename = f"missions/Mavic3_{ts_str}.kmz"
    wpml_content = generate_native_wpml(coords, CONFIG)

    with zipfile.ZipFile(kmz_filename, 'w') as kmz:
        kmz.writestr("wpmz/waylines.wpml", wpml_content)
        kmz.writestr("wpmz/template.kml", wpml_content)
    
    print(f"Mission Generated in Feet: {kmz_filename}")

if __name__ == "__main__":
    create_mission()