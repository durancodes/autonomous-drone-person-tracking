from dronekit import connect, VehicleMode
import cv2
from ultralytics import YOLO
import pyttsx3
import threading
import time
import math

# === TTS Setup ===
engine = pyttsx3.init()
def speak(text):
    print("🗣", text)
    engine.say(text)
    engine.runAndWait()

# === Connect to drone ===
print("Connecting to drone...")
vehicle = connect('127.0.0.1:14550', wait_ready=True, timeout=90, heartbeat_timeout=90)
print("✅ Drone connected.")

# === RC Override ===
def send_rc_override(yaw_value, pitch_value=1500):
    yaw_value = max(1000, min(2000, int(yaw_value)))
    pitch_value = max(1000, min(2000, int(pitch_value)))
    vehicle.channels.overrides = {'4': yaw_value, '2': pitch_value}
    print(f"[RC] Yaw: {yaw_value} | Pitch: {pitch_value}")

# === Arm and Takeoff Thread ===
def arm_takeoff_thread(alt):
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    speak("Arming")
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    speak("Taking off")
    vehicle.simple_takeoff(alt)
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"🚁 Altitude: {current_alt:.2f}m")
        if current_alt >= alt * 0.95:
            speak("Reached target altitude")
            break
        time.sleep(1)

takeoff_thread = threading.Thread(target=arm_takeoff_thread, args=(2.5,))
takeoff_thread.start()

# === GPS Distance + Bearing Functions ===
def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371e3
    φ1, φ2 = map(math.radians, (lat1, lat2))
    Δφ = math.radians(lat2 - lat1)
    Δλ = math.radians(lon2 - lon1)
    a = math.sin(Δφ/2)*2 + math.cos(φ1)*math.cos(φ2)*math.sin(Δλ/2)*2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def calculate_bearing(lat1, lon1, lat2, lon2):
    φ1, φ2 = map(math.radians, (lat1, lat2))
    Δλ = math.radians(lon2 - lon1)
    y = math.sin(Δλ) * math.cos(φ2)
    x = math.cos(φ1)*math.sin(φ2) - math.sin(φ1)*math.cos(φ2)*math.cos(Δλ)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360) % 360

# === Simulated or Real GPS of Drone B (change these values) ===
target_drone_lat = 12.9715987  # Replace with Drone B lat
target_drone_lon = 77.594566   # Replace with Drone B lon

# === YOLO and Camera Setup ===
model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Camera failed to open")
    exit()
else:
    print("✅ Camera found at index 0")

frame_width, frame_height = 640, 480
frame_center = (frame_width // 2, frame_height // 2)
selected_center = None
selection_enabled = True

# === PID setup ===
Kp = 0.5
last_time = time.time()
last_pid_time = time.time()

# === GPS Setup ===
GPS_UPDATE_INTERVAL = 0.2
last_gps_time = time.time()
last_gps_read_time = time.time()
gps_rate = 0
loc_cache = vehicle.location

# === Mouse click ===
def select_person(event, x, y, flags, param):
    global selected_center, selection_enabled
    if event == cv2.EVENT_LBUTTONDOWN and selection_enabled:
        selected_center = (x, y)
        selection_enabled = False
        speak("Target selected")

cv2.namedWindow("YOLO TRACKER")
cv2.setMouseCallback("YOLO TRACKER", select_person)

# === Main Loop ===
while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠ Frame error.")
        break

    now = time.time()
    cam_dt = now - last_time
    last_time = now
    fps = 1 / cam_dt if cam_dt > 0 else 0

    pid_dt = now - last_pid_time
    pid_rate = 1 / pid_dt if pid_dt > 0 else 0
    last_pid_time = now

    if now - last_gps_time >= GPS_UPDATE_INTERVAL:
        loc_cache = vehicle.location
        gps_rate = 1 / (now - last_gps_read_time)
        last_gps_read_time = now
        last_gps_time = now

    frame = cv2.resize(frame, (frame_width, frame_height))
    results = model(frame, verbose=False)

    target_box = None
    min_dist = float('inf')

    for r in results:
        for box in r.boxes:
            if int(box.cls[0]) != 0:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            if selected_center:
                dist = (cx - selected_center[0]) ** 2 + (cy - selected_center[1]) ** 2
                if dist < min_dist:
                    min_dist = dist
                    target_box = (x1, y1, x2, y2)

    if target_box:
        x1, y1, x2, y2 = target_box
        person_center_x = (x1 + x2) // 2
        error = person_center_x - frame_center[0]
        yaw_rc = 1500 + Kp * error
        send_rc_override(yaw_rc, 1500)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    else:
        send_rc_override(1500, 1500)

    # === Compute distance & heading to Drone B ===
    me = loc_cache.global_frame
    dist_to_b = haversine_distance(me.lat, me.lon, target_drone_lat, target_drone_lon)
    heading_to_b = calculate_bearing(me.lat, me.lon, target_drone_lat, target_drone_lon)

    # === Draw UI elements ===
    cv2.line(frame, (frame_center[0] - 10, frame_center[1]),
             (frame_center[0] + 10, frame_center[1]), (255, 255, 255), 2)
    cv2.line(frame, (frame_center[0], frame_center[1] - 10),
             (frame_center[0], frame_center[1] + 10), (255, 255, 255), 2)

    cv2.putText(frame, f"FPS: {fps:.2f} | PID Rate: {pid_rate:.2f} | GPS: {gps_rate:.2f} Hz", 
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    cv2.putText(frame,
                f"Lat: {me.lat:.6f} Lon: {me.lon:.6f}",
                (10, frame_height - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(frame,
                f"Alt: {me.alt:.2f} RelAlt: {loc_cache.global_relative_frame.alt:.2f}",
                (10, frame_height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(frame,
                f"To Drone B → {dist_to_b/1e3:.2f} km | Heading: {heading_to_b:.1f}°",
                (10, frame_height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    cv2.imshow("YOLO TRACKER", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        speak("Landing")
        vehicle.mode = VehicleMode("LAND")
        vehicle.channels.overrides = {}
        time.sleep(3)
        vehicle.armed = False
        break
    elif key == ord('s'):
        selected_center = None
        selection_enabled = True
        speak("Selection reset")
