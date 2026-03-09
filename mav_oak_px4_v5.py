import time
import math

import depthai as dai
import blobconverter
from pymavlink import mavutil

import numpy as np
import math

PORT = "/dev/ttyTHS1"       # Jetson UART to Cube
BAUD = 57600
SYS_ID = 42                 # Jetson MAVLink system id
COMP_ID = 191               # Onboard computer component id

# I set to person to test. we can change it to cars as well later on 
TARGET_LABEL = {"person", "car"}

PRINT_HZ = 5.0              # how often to print to terminal
HEARTBEAT_HZ = 1.0          # Jetson heartbeat to PX4



labels = [
    "background","aeroplane","bicycle","bird","boat","bottle","bus","car","cat","chair",
    "cow","diningtable","dog","horse","motorbike","person","pottedplant","sheep","sofa","train","tvmonitor"
]


mav = mavutil.mavlink_connection(
    PORT,
    baud=BAUD,
    source_system=SYS_ID,
    source_component=COMP_ID,
    autoreconnect=True,
)

# Mavelink channel to send the calculated obstacle coordinates 
#_____________________________________________________________
ADSB_HZ = 2.0
last_adsb = 0.0
ADSB_ICAO = 0xABC001  # this can be any address. it is just to set one for each target so QGC knows this info for the same target 

TARGET_VANISH = 2.0
last_seen = 0.0
vanished = True



def deg_to_e7(x): # mavlink adsb dont understand degree. it excpect the degree to be times 10^7
    return int(round(x * 1e7)) 

def callsign_8(name): #adsb in mavlink has a limitation of 8 char. so we are avoiding the rest of the obstacle name
    s = name[:8].ljust(8)
    return s.encode("ascii") + b"\0"  

ADSB_FLAGS = (   # these flags i used somewhere else as well to tell QGC these blocks are important for me so dont ignore them
    mavutil.mavlink.ADSB_FLAGS_VALID_COORDS |
    mavutil.mavlink.ADSB_FLAGS_VALID_ALTITUDE |
    mavutil.mavlink.ADSB_FLAGS_VALID_HEADING |
    mavutil.mavlink.ADSB_FLAGS_VALID_VELOCITY |
    mavutil.mavlink.ADSB_FLAGS_VALID_CALLSIGN |
    getattr(mavutil.mavlink, "ADSB_FLAGS_SIMULATED", 64)
)

ADSB_ALT_TYPE = getattr(mavutil.mavlink, "ADSB_ALTITUDE_TYPE_GEOMETRIC", 1) # basically teaching QGC how to read the altitude

ADSB_EMITTER_POINT_OBSTACLE = getattr(
    mavutil.mavlink,
    "ADSB_EMITTER_TYPE_POINT_OBSTACLE",
    getattr(mavutil.mavlink, "ADSB_EMITTER_TYPE_UAV", 14)
)

#_____________________________________


pipeline = dai.Pipeline()

# RGB + Stereo for spatial detections
cam_rgb = pipeline.createColorCamera()
cam_rgb.setPreviewSize(300, 300)
cam_rgb.setInterleaved(False)

mono_left = pipeline.createMonoCamera()
mono_right = pipeline.createMonoCamera()
mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

stereo = pipeline.createStereoDepth()
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

det_nn = pipeline.createMobileNetSpatialDetectionNetwork()
det_nn.setConfidenceThreshold(0.5)     
det_nn.setBoundingBoxScaleFactor(0.5)
det_nn.setDepthLowerThreshold(100)     # mm
det_nn.setDepthUpperThreshold(10000)   # mm
det_nn.setBlobPath(blobconverter.from_zoo(name="mobilenet-ssd", shaves=6))

# IMU (camera quaternion)
imu = pipeline.createIMU()
imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)  # 100 Hz
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)

# Outputs
xout_nn = pipeline.createXLinkOut()
xout_nn.setStreamName("nn")

xout_imu = pipeline.createXLinkOut()
xout_imu.setStreamName("imu")

# Links
mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

cam_rgb.preview.link(det_nn.input)
stereo.depth.link(det_nn.inputDepth)
det_nn.out.link(xout_nn.input)

imu.out.link(xout_imu.input)

# ==============================
# Helpers
# ==============================
def norm_deg_180(deg):
    #Normalize angle to [-180, 180).
    return (deg + 180.0) % 360.0 - 180.0

def get_quat_xyzw_from_packet(packet):
    """
    Return camera quaternion as (x,y,z,w)
    """
    if not hasattr(packet, "rotationVector"):
        return None

    rv = packet.rotationVector

    # Most common DepthAI field names
    if all(hasattr(rv, a) for a in ("i", "j", "k", "real")):
        return (float(rv.i), float(rv.j), float(rv.k), float(rv.real))

    if all(hasattr(rv, a) for a in ("x", "y", "z", "w")):
        return (float(rv.x), float(rv.y), float(rv.z), float(rv.w))

    if all(hasattr(rv, a) for a in ("x", "y", "z", "real")):
        return (float(rv.x), float(rv.y), float(rv.z), float(rv.real))

    return None

#-------------------------------------------------------------------

EARTH_R = 6378137.0  # meters

def quat_xyzw_to_R_wb(q_xyzw):
    """Quaternion (x,y,z,w) -> rotation matrix mapping body/camera -> world."""
    x, y, z, w = q_xyzw
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        raise ValueError("Quaternion has zero norm.")
    x, y, z, w = x/n, y/n, z/n, w/n

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ], dtype=float)

def extract_yaw_pitch_roll_ZYX(R):
    """Extract yaw(Z), pitch(Y), roll(X) from rotation matrix using ZYX convention."""
    r20 = float(R[2, 0])
    r20 = max(-1.0, min(1.0, r20))
    pitch = math.asin(-r20)

    if abs(math.cos(pitch)) < 1e-8:
        yaw = 0.0
        roll = math.atan2(-R[0, 1], R[1, 1])
    else:
        yaw  = math.atan2(R[1, 0], R[0, 0])
        roll = math.atan2(R[2, 1], R[2, 2])

    return yaw, pitch, roll

def Rx(roll):
    c, s = math.cos(roll), math.sin(roll)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s,  c]], dtype=float)

def Ry(pitch):
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]], dtype=float)

def Rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], dtype=float)

def offsets_ne_to_latlon(lat0_deg, lon0_deg, north_m, east_m):
    """Local tangent approximation (excellent for 0–10 m)."""
    lat0 = math.radians(lat0_deg)
    dlat = north_m / EARTH_R
    dlon = east_m  / (EARTH_R * math.cos(lat0))
    return (lat0_deg + math.degrees(dlat), lon0_deg + math.degrees(dlon))

def target_gps_from_oak_px4(lat0_deg, lon0_deg, yaw_deg, quat_xyzw, x_m, y_m, z_m):
    """
    lat0/lon0: camera GPS (deg)
    yaw_deg: PX4 yaw/heading in degrees (0=N, 90=E)  <-- if your PX4 is NED yaw, see note below
    quat_xyzw: OAK quaternion (x,y,z,w) for camera orientation (yaw will be replaced)
    x_m,y_m,z_m: target in camera frame (meters). axes: X right, Y down, Z forward
    """
    R_wb = quat_xyzw_to_R_wb(quat_xyzw)

    # Get pitch/roll from OAK orientation, ignore its yaw
    _, pitch, roll = extract_yaw_pitch_roll_ZYX(R_wb)

    # Replace yaw with PX4 yaw
    yaw = math.radians(yaw_deg)
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    v_cam = np.array([x_m, y_m, z_m], dtype=float)
    v_world = R @ v_cam

    # Interpret world axes as:
    #   X = North, Y = East (so v_world[0]=north, v_world[1]=east)
    north_m = float(v_world[0])
    east_m  = float(v_world[1])

    return offsets_ne_to_latlon(lat0_deg, lon0_deg, north_m, east_m)

#-------------------------------------------------------------------
# State holders
latest_quat = None          # from OAK IMU

latest_obj = None                    # dict: {"label": str, "x_mm": int, "y_mm": int, "z_mm": int}
latest_obj_t = None

latest_yaw = None                # from PX4 ATTITUDE
latest_att_t = None

latest_gps = None                    # dict: {"lat":..., "lon":..., "alt"::...}
latest_gps_t = None
last_print = 0.0
last_hb = 0.0

print(f"Opening MAVLink on {PORT} @ {BAUD}...")
print("Starting OAK pipeline...")

with dai.Device(pipeline) as device:
    q_nn = device.getOutputQueue("nn", maxSize=4, blocking=False)
    q_imu = device.getOutputQueue("imu", maxSize=30, blocking=False)

    print("Fusion bridge running.")
    print("Streaming:")
    print("  - OAK camera quaternion (x,y,z,w)")
    print("  - OAK object xyz (mm, camera frame)")
    print("  - PX4 yaw (deg)")
    print("  - PX4 GPS (lat, lon, alt)")
    print("Ctrl+C to stop.\n")

    while True:
        now = time.time()

        
        # 1) Send Jetson heartbeat to PX4
      
        if now - last_hb >= (1.0 / HEARTBEAT_HZ):
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE
            )
            last_hb = now

        
        # 2) Read OAK IMU quaternion (camera orientation)
      
        in_imu = q_imu.tryGet()
        if in_imu is not None:
            # DepthAI batches packets; use the most recent one in the batch
            for pkt in in_imu.packets:
                quat = get_quat_xyzw_from_packet(pkt)
                if quat is not None:
                    latest_quat = quat

        # 3) Read OAK detections (object xyz in camera frame)
       
        in_nn = q_nn.tryGet()
        if in_nn is not None:
            best = None  # nearest valid detection
            for det in in_nn.detections:
                label = labels[det.label] if det.label < len(labels) else str(det.label)

                if TARGET_LABEL is not None and label not in TARGET_LABEL:
                    continue

                x_mm = int(det.spatialCoordinates.x )  
                y_mm = int(det.spatialCoordinates.y )  
                z_mm = int(det.spatialCoordinates.z ) 
                

                # Skip invalid depth
                if z_mm <= 0:
                    continue

                if best is None or z_mm < best["z_mm"]:
                    best = {
                        "label": label,
                        "x_mm": x_mm,
                        "y_mm": y_mm,
                        "z_mm": z_mm,
                    }

            # Update latest object only if we saw one this frame
            if best is not None:
                latest_obj = best
                latest_obj_t = now

                last_seen = now
                vanished = False

        # 4) Read PX4 MAVLink messages (yaw + GPS)
        #    Drain a few messages each loop to keep up.
       
        for _ in range(20):
            msg = mav.recv_match(
                type=["ATTITUDE", "GLOBAL_POSITION_INT"],
                blocking=False
            )
            if msg is None:
                break

            mtype = msg.get_type()

            if mtype == "ATTITUDE":
                yaw = math.degrees(msg.yaw)
                latest_yaw = norm_deg_180(yaw)
                latest_att_t = now

            elif mtype == "GLOBAL_POSITION_INT":
                # Preferred for lat/lon/alt + relative altitude
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0              # AMSL

                latest_gps = {
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                }
                latest_gps_t = now

        # --- VANISH logic (runs every loop, not only print) ---
        if not vanished and (now - last_seen) > TARGET_VANISH:
            # 1) Stop using the old detection
            latest_obj = None
            latest_obj_t = None

            # 2) Tell QGC this ADSB target is no longer valid
            flags_no_coords = ADSB_FLAGS & ~mavutil.mavlink.ADSB_FLAGS_VALID_COORDS

            mav.mav.adsb_vehicle_send(
                ADSB_ICAO,
                0, 0,                      # invalid lat/lon
                ADSB_ALT_TYPE,
                0,
                0,
                0, 0,
                callsign_8(""),            # empty callsign is fine
                ADSB_EMITTER_POINT_OBSTACLE,
                10,                        # tslc big (stale)
                flags_no_coords,
                0
            )

            vanished = True
            #print("VANISH: cleared target + sent invalid ADSB")

        # 5) Print summary line on Jetson (throttled)
        if now - last_print >= (1.0 / PRINT_HZ):
            # Camera quaternion
            if latest_quat is None:
                cam_quat_str = "quat=WAIT"
            else:
                qx, qy, qz, qw = latest_quat
                cam_quat_str = f"quat=({qx:+.5f},{qy:+.5f},{qz:+.5f},{qw:+.5f})"

            # Object XYZ
            if latest_obj is None:
                obj_str = "obj=NONE"
            else:
                obj_str = (
                    f"obj={latest_obj['label']} "
                    f"target_xyz=({latest_obj['x_mm']},{latest_obj['y_mm']},{latest_obj['z_mm']}) "
                )

            # Yaw
            if latest_yaw is None:
                yaw_str = "yaw=WAIT"
            else:
                yaw_str = f"yaw={latest_yaw:+.2f}"

            # GPS
            if latest_gps is None:
                gps_str = "gps=WAIT"
            else:
                lat = latest_gps["lat"]
                lon = latest_gps["lon"]
                alt = latest_gps["alt"]
                sats = latest_gps.get("sats")

                gps_str = (
                    f"gps=({lat:.7f},{lon:.7f}) alt={alt:.2f}"
                )
            # --- Target GPS calculation ---
            tgt_str = "tgt=WAIT"
            if latest_quat is not None and latest_obj is not None and latest_yaw is not None and latest_gps is not None:

                x_mm = latest_obj["x_mm"] 
                y_mm = latest_obj["y_mm"] 
                z_mm = latest_obj["z_mm"] 

                x_m = x_mm / 1000.0
                y_m = y_mm / 1000.0
                z_m = z_mm / 1000.0

                cam_lat = latest_gps["lat"]
                cam_lon = latest_gps["lon"]
                yaw_deg = latest_yaw   # from PX4 attitude

                tlat, tlon = target_gps_from_oak_px4(
                    cam_lat, cam_lon,
                    yaw_deg=yaw_deg,
                    quat_xyzw=latest_quat,
                    x_m=x_m, y_m=y_m, z_m=z_m
                )
            #_____________________________
                if now - last_adsb >= (1.0 / ADSB_HZ):
                    # callsign drives your QGC icon selection

                    if latest_obj["label"] == "person":
                        cs = "PERSON"
                    elif latest_obj["label"] == "car":
                        cs = "CAR"
                    else: cs = "TARGET"
                    #cs = "PERSON1" if latest_obj["label"] == "person" else "CAR1"

                    mav.mav.adsb_vehicle_send(
                        ADSB_ICAO,
                        deg_to_e7(tlat),
                        deg_to_e7(tlon),
                        ADSB_ALT_TYPE,
                        int(round(z_m * 1000)),          # this is altitude but i am using it to send distance
                        int(round((latest_yaw % 360.0) * 100)),        # centi-deg
                        0, 0,                                          # horiz/vert vel cm/s
                        callsign_8(cs),
                        ADSB_EMITTER_POINT_OBSTACLE,
                        1,
                        ADSB_FLAGS,
                        0
                    )

                #    print(f"TX ADSB -> PX4: {cs} {tlat:.7f},{tlon:.7f}")
                    last_adsb = now
                

            #______________________________
                tgt_str = f"tgt=({tlat:.7f},{tlon:.7f})"

            print(f"{cam_quat_str} | {obj_str} | {yaw_str} | {gps_str} | {tgt_str}")

            last_print = now

