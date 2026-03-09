import time
import math
import numpy as np

import depthai as dai
import blobconverter
from pymavlink import mavutil


# ============================================================
# Config
# ============================================================
PORT = "/dev/ttyTHS1"   # Jetson UART to Cube
BAUD = 57600
SYS_ID = 42             # Jetson MAVLink system id
COMP_ID = 191           # Onboard computer component id

TARGET_LABELS = {"person", "car"}   # None = allow all labels

PRINT_HZ = 5.0
HEARTBEAT_HZ = 1.0
ADSB_HZ = 2.0

TRACK_TIMEOUT = 2.0                 # seconds before stale track is removed
TRACK_MATCH_DIST_MM = 1500          # 3D distance gate for matching detections
ADSB_ICAO_BASE = 0xABC000           # each track gets ICAO = BASE + track_id

EARTH_R = 6378137.0                 # meters


# ============================================================
# State
# ============================================================
tracks = {}                         # track_id -> dict
next_track_id = 1

latest_quat = None                  # tuple (x, y, z, w)
latest_yaw = None                   # degrees
latest_gps = None                   # {"lat":..., "lon":..., "alt":...}

last_print = 0.0
last_hb = 0.0
last_adsb = 0.0


# ============================================================
# Labels
# ============================================================
labels = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle",
    "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]


# ============================================================
# MAVLink
# ============================================================
mav = mavutil.mavlink_connection(
    PORT,
    baud=BAUD,
    source_system=SYS_ID,
    source_component=COMP_ID,
    autoreconnect=True,
)

ADSB_FLAGS = (
    mavutil.mavlink.ADSB_FLAGS_VALID_COORDS |
    mavutil.mavlink.ADSB_FLAGS_VALID_ALTITUDE |
    mavutil.mavlink.ADSB_FLAGS_VALID_HEADING |
    mavutil.mavlink.ADSB_FLAGS_VALID_VELOCITY |
    mavutil.mavlink.ADSB_FLAGS_VALID_CALLSIGN |
    getattr(mavutil.mavlink, "ADSB_FLAGS_SIMULATED", 64)
)

ADSB_ALT_TYPE = getattr(
    mavutil.mavlink,
    "ADSB_ALTITUDE_TYPE_GEOMETRIC",
    1
)

ADSB_EMITTER_POINT_OBSTACLE = getattr(
    mavutil.mavlink,
    "ADSB_EMITTER_TYPE_POINT_OBSTACLE",
    getattr(mavutil.mavlink, "ADSB_EMITTER_TYPE_UAV", 14)
)


# ============================================================
# Helpers
# ============================================================
def deg_to_e7(x):
    """Convert degrees to MAVLink E7 integer format."""
    return int(round(x * 1e7))


def callsign_8(name):
    """
    ADS-B callsign field is limited.
    Keep 8 chars and append null terminator.
    """
    s = str(name)[:8].ljust(8)
    return s.encode("ascii", errors="ignore") + b"\0"


def adsb_callsign_from_label(label, idx=1):
    """
    Build callsign like PERSON1, CAR2, etc.
    Limited to 8 chars before null terminator.
    """
    s = (label or "TARGET").upper()
    s = "".join(ch for ch in s if ch.isalnum())

    suffix = str(idx)
    max_base = max(1, 8 - len(suffix))
    s = (s[:max_base] + suffix)[:8]
    return s


def norm_deg_180(deg):
    """Normalize angle to [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


def get_quat_xyzw_from_packet(packet):
    """Return camera quaternion as (x, y, z, w)."""
    if not hasattr(packet, "rotationVector"):
        return None

    rv = packet.rotationVector

    if all(hasattr(rv, a) for a in ("i", "j", "k", "real")):
        return (float(rv.i), float(rv.j), float(rv.k), float(rv.real))

    if all(hasattr(rv, a) for a in ("x", "y", "z", "w")):
        return (float(rv.x), float(rv.y), float(rv.z), float(rv.w))

    if all(hasattr(rv, a) for a in ("x", "y", "z", "real")):
        return (float(rv.x), float(rv.y), float(rv.z), float(rv.real))

    return None


def make_track_icao(track_id):
    """Unique ICAO address for each track."""
    return ADSB_ICAO_BASE + int(track_id)


def det_distance_mm(a, b):
    """3D Euclidean distance between detection a and track/detection b in mm."""
    dx = a["x_mm"] - b["x_mm"]
    dy = a["y_mm"] - b["y_mm"]
    dz = a["z_mm"] - b["z_mm"]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def send_adsb_delete(icao):
    """
    Send an invalid ADS-B packet so QGC drops or marks the target stale.
    """
    flags_no_coords = ADSB_FLAGS & ~mavutil.mavlink.ADSB_FLAGS_VALID_COORDS

    mav.mav.adsb_vehicle_send(
        icao,
        0, 0,                      # invalid lat/lon
        ADSB_ALT_TYPE,
        0,
        0,
        0, 0,
        callsign_8("DEL"),
        ADSB_EMITTER_POINT_OBSTACLE,
        10,
        flags_no_coords,
        0
    )


def build_visible_index_map(tracks_dict):
    """
    Assign display index only for currently active tracks.
    Numbering resets from 1 for each label.
    Example:
      person -> PERSON1, PERSON2, PERSON3
      car    -> CAR1, CAR2
    Sorted left-to-right in camera frame using x_mm.
    """
    visible_index = {}

    labels_present = sorted(set(tr["label"] for tr in tracks_dict.values()))
    for label in labels_present:
        same_label = [(tid, tr) for tid, tr in tracks_dict.items() if tr["label"] == label]

        # Left-to-right, then depth
        same_label.sort(key=lambda item: (item[1]["x_mm"], item[1]["z_mm"]))

        for idx, (tid, tr) in enumerate(same_label, start=1):
            visible_index[tid] = idx

    return visible_index


def quat_xyzw_to_R_wb(q_xyzw):
    """Quaternion (x,y,z,w) -> rotation matrix mapping camera/body -> world."""
    x, y, z, w = q_xyzw
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0:
        raise ValueError("Quaternion has zero norm.")

    x, y, z, w = x / n, y / n, z / n, w / n

    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w)],
        [    2 * (x * y + z * w), 1 - 2 * (x * x + z * z),     2 * (y * z - x * w)],
        [    2 * (x * z - y * w),     2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]
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
        yaw = math.atan2(R[1, 0], R[0, 0])
        roll = math.atan2(R[2, 1], R[2, 2])

    return yaw, pitch, roll


def Rx(roll):
    c, s = math.cos(roll), math.sin(roll)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s,  c]
    ], dtype=float)


def Ry(pitch):
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]
    ], dtype=float)


def Rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ], dtype=float)


def offsets_ne_to_latlon(lat0_deg, lon0_deg, north_m, east_m):
    """Local tangent approximation."""
    lat0 = math.radians(lat0_deg)
    dlat = north_m / EARTH_R
    dlon = east_m / (EARTH_R * math.cos(lat0))
    return (
        lat0_deg + math.degrees(dlat),
        lon0_deg + math.degrees(dlon)
    )


def target_gps_from_oak_px4(lat0_deg, lon0_deg, yaw_deg, quat_xyzw, x_m, y_m, z_m):
    """
    lat0/lon0: camera GPS (deg)
    yaw_deg: PX4 yaw/heading in degrees
    quat_xyzw: OAK quaternion (x,y,z,w), using pitch/roll from OAK, yaw from PX4
    x_m,y_m,z_m: target in camera frame (meters)
    """
    R_wb = quat_xyzw_to_R_wb(quat_xyzw)

    # Keep pitch/roll from OAK, replace yaw with PX4 yaw
    _, pitch, roll = extract_yaw_pitch_roll_ZYX(R_wb)
    yaw = math.radians(yaw_deg)

    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    v_cam = np.array([x_m, y_m, z_m], dtype=float)
    v_world = R @ v_cam

    # Interpret world axes as X=north, Y=east
    north_m = float(v_world[0])
    east_m = float(v_world[1])

    return offsets_ne_to_latlon(lat0_deg, lon0_deg, north_m, east_m)


# ============================================================
# DepthAI pipeline
# ============================================================
pipeline = dai.Pipeline()

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

imu = pipeline.createIMU()
imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)

xout_nn = pipeline.createXLinkOut()
xout_nn.setStreamName("nn")

xout_imu = pipeline.createXLinkOut()
xout_imu.setStreamName("imu")

mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

cam_rgb.preview.link(det_nn.input)
stereo.depth.link(det_nn.inputDepth)
det_nn.out.link(xout_nn.input)

imu.out.link(xout_imu.input)


# ============================================================
# Main
# ============================================================
print("Starting OAK pipeline...")

with dai.Device(pipeline) as device:
    q_nn = device.getOutputQueue("nn", maxSize=4, blocking=False)
    q_imu = device.getOutputQueue("imu", maxSize=30, blocking=False)

    while True:
        now = time.time()

        # --------------------------------------------------------
        # 1) Heartbeat from Jetson to PX4
        # --------------------------------------------------------
        if now - last_hb >= (1.0 / HEARTBEAT_HZ):
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE
            )
            last_hb = now

        # --------------------------------------------------------
        # 2) Read OAK IMU quaternion
        # --------------------------------------------------------
        in_imu = q_imu.tryGet()
        if in_imu is not None:
            for pkt in in_imu.packets:
                quat = get_quat_xyzw_from_packet(pkt)
                if quat is not None:
                    latest_quat = quat

        # --------------------------------------------------------
        # 3) Read all OAK detections and track them
        # --------------------------------------------------------
        in_nn = q_nn.tryGet()
        if in_nn is not None:
            detections = []

            for det in in_nn.detections:
                label = labels[det.label] if det.label < len(labels) else str(det.label)

                if TARGET_LABELS is not None and label not in TARGET_LABELS:
                    continue

                x_mm = int(det.spatialCoordinates.x)
                y_mm = int(det.spatialCoordinates.y)
                z_mm = int(det.spatialCoordinates.z)

                if z_mm <= 0:
                    continue

                detections.append({
                    "label": label,
                    "x_mm": x_mm,
                    "y_mm": y_mm,
                    "z_mm": z_mm,
                })

            # Build candidate matches between detections and existing tracks
            candidate_pairs = []

            for di, det in enumerate(detections):
                for tid, tr in tracks.items():
                    if (now - tr["last_seen"]) > TRACK_TIMEOUT:
                        continue

                    if tr["label"] != det["label"]:
                        continue

                    d_mm = det_distance_mm(det, tr)
                    if d_mm <= TRACK_MATCH_DIST_MM:
                        candidate_pairs.append((d_mm, di, tid))

            # Greedy nearest-neighbor assignment
            candidate_pairs.sort(key=lambda x: x[0])

            assigned_det = set()
            assigned_tid = set()
            det_to_tid = {}

            for d_mm, di, tid in candidate_pairs:
                if di in assigned_det or tid in assigned_tid:
                    continue
                det_to_tid[di] = tid
                assigned_det.add(di)
                assigned_tid.add(tid)

            # Update matched tracks, create new tracks for unmatched detections
            for di, det in enumerate(detections):
                if di in det_to_tid:
                    tid = det_to_tid[di]
                else:
                    tid = next_track_id
                    next_track_id += 1
                    tracks[tid] = {
                        "id": tid,
                        "icao": make_track_icao(tid),
                    }

                tracks[tid].update({
                    "id": tid,
                    "icao": make_track_icao(tid),
                    "label": det["label"],
                    "x_mm": det["x_mm"],
                    "y_mm": det["y_mm"],
                    "z_mm": det["z_mm"],
                    "last_seen": now,
                })

        # --------------------------------------------------------
        # 4) Read PX4 MAVLink messages
        # --------------------------------------------------------
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

            elif mtype == "GLOBAL_POSITION_INT":
                latest_gps = {
                    "lat": msg.lat / 1e7,
                    "lon": msg.lon / 1e7,
                    "alt": msg.alt / 1000.0,    # meters AMSL
                }

        # --------------------------------------------------------
        # 5) Remove stale tracks
        # --------------------------------------------------------
        for tid in list(tracks.keys()):
            if (now - tracks[tid]["last_seen"]) > TRACK_TIMEOUT:
                send_adsb_delete(tracks[tid]["icao"])
                del tracks[tid]

        # --------------------------------------------------------
        # 6) Send ADS-B for all active tracks
        # --------------------------------------------------------
        if now - last_adsb >= (1.0 / ADSB_HZ):
            if latest_quat is not None and latest_yaw is not None and latest_gps is not None:
                cam_lat = latest_gps["lat"]
                cam_lon = latest_gps["lon"]
                yaw_deg = latest_yaw

                visible_index_map = build_visible_index_map(tracks)

                for tid in sorted(tracks.keys()):
                    tr = tracks[tid]

                    x_m = tr["x_mm"] / 1000.0
                    y_m = tr["y_mm"] / 1000.0
                    z_m = tr["z_mm"] / 1000.0

                    tlat, tlon = target_gps_from_oak_px4(
                        cam_lat,
                        cam_lon,
                        yaw_deg=yaw_deg,
                        quat_xyzw=latest_quat,
                        x_m=x_m,
                        y_m=y_m,
                        z_m=z_m
                    )

                    tr["tlat"] = tlat
                    tr["tlon"] = tlon

                    display_idx = visible_index_map[tid]
                    cs_text = adsb_callsign_from_label(tr["label"], display_idx)

                    mav.mav.adsb_vehicle_send(
                        tr["icao"],                               # stable ICAO
                        deg_to_e7(tlat),
                        deg_to_e7(tlon),
                        ADSB_ALT_TYPE,
                        int(tr["z_mm"]),                          # using distance as altitude for testing
                        int(round((latest_yaw % 360.0) * 100)),  # heading centi-deg
                        0,
                        0,
                        callsign_8(cs_text),
                        ADSB_EMITTER_POINT_OBSTACLE,
                        min(255, max(1, int(now - tr["last_seen"]))),
                        ADSB_FLAGS,
                        0
                    )

            last_adsb = now

        # --------------------------------------------------------
        # 7) Print summary
        # --------------------------------------------------------
        if now - last_print >= (1.0 / PRINT_HZ):
            if latest_quat is None:
                cam_quat_str = "quat=WAIT"
            else:
                qx, qy, qz, qw = latest_quat
                cam_quat_str = f"quat=({qx:+.5f},{qy:+.5f},{qz:+.5f},{qw:+.5f})"

            if latest_yaw is None:
                yaw_str = "yaw=WAIT"
            else:
                yaw_str = f"yaw={latest_yaw:+.2f}"

            if latest_gps is None:
                gps_str = "gps=WAIT"
            else:
                gps_str = (
                    f"gps=({latest_gps['lat']:.7f},{latest_gps['lon']:.7f}) "
                    f"alt={latest_gps['alt']:.2f}"
                )

            visible_index_map = build_visible_index_map(tracks)

            if not tracks:
                obj_str = "obj=NONE"
            else:
                obj_parts = []
                for tid in sorted(tracks.keys()):
                    tr = tracks[tid]
                    obj_parts.append(
                        f"{tr['label']}#{visible_index_map[tid]}=({tr['x_mm']},{tr['y_mm']},{tr['z_mm']})"
                    )
                obj_str = "obj=" + " ; ".join(obj_parts)

            tgt_parts = []
            for tid in sorted(tracks.keys()):
                tr = tracks[tid]
                if "tlat" in tr and "tlon" in tr:
                    tgt_parts.append(
                        f"{tr['label']}#{visible_index_map[tid]}=({tr['tlat']:.7f},{tr['tlon']:.7f})"
                    )

            tgt_str = "tgt=" + (" ; ".join(tgt_parts) if tgt_parts else "WAIT")

            print(f"{cam_quat_str} | {obj_str} | {yaw_str} | {gps_str} | {tgt_str}")

            last_print = now
