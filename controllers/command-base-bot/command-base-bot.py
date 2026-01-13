from controller import Robot
import numpy as np
import cv2
import os
import time
import math

# ============================
# ROBOT SETUP
# ============================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28

# ============================
# THRESHOLDS
# ============================
SHAPE_CHECK_AREA = {"ball": 400, "cylinder": 350, "cone": 400}
STOP_AREA = {"ball":780, "cylinder": 900, "cone": 900}
FRONT_STOP = {"ball": 78, "cylinder": 85, "cone": 90}
CENTER_TOLERANCE = 15

# ============================
# REJECT MEMORY
# ============================
REJECT_IGNORE_TIME = 6.0
REJECT_CX_WINDOW = 100
REJECT_AREA_RATIO = 0.40

rejected_cx = None
rejected_area = None
reject_until = 0.0

# ============================
# SEARCH PATTERN (NEW)
# ============================
ROTATION_TIME = 6.0      # ~1 full rotation
MAX_ROTATIONS = 2        # after 2 rotations
FORWARD_TIME = 6.0       # go straight 6s

rotation_start_time = robot.getTime()
rotation_count = 0
FORWARD_SEARCH = False
FORWARD_UNTIL = 0.0

# ============================
# SEARCH MODE (WRONG SHAPE)
# ============================
SEARCH_MODE = False
SEARCH_UNTIL = 0.0

# ============================
# OBSTACLE ESCAPE
# ============================
OBSTACLE_ESCAPE = False
OBSTACLE_ESCAPE_UNTIL = 0.0
OBSTACLE_ESCAPE_MODE = None

# ============================
# SHAPE LOCK
# ============================
SHAPE_LOCKED = False

# ============================
# DEVICES
# ============================
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

camera = robot.getDevice("camera")
camera.enable(timestep)
WIDTH = camera.getWidth()
HEIGHT = camera.getHeight()
FRAME_CENTER = WIDTH // 2

ps = []
for i in range(8):
    s = robot.getDevice('ps' + str(i))
    s.enable(timestep)
    ps.append(s)

# ============================
# HELPERS
# ============================
def stop_robot():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def read_front_left_right():
    front = max(ps[0].getValue(), ps[7].getValue())
    left  = max(ps[5].getValue(), ps[6].getValue())
    right = max(ps[1].getValue(), ps[2].getValue())
    return front, left, right

def avoid_obstacles():
    global OBSTACLE_ESCAPE, OBSTACLE_ESCAPE_UNTIL, OBSTACLE_ESCAPE_MODE

    front, left, right = read_front_left_right()
    TH = 80
    now = robot.getTime()

    if OBSTACLE_ESCAPE:
        if OBSTACLE_ESCAPE_MODE == "front":
            left_motor.setVelocity(0.6 * MAX_SPEED)
            right_motor.setVelocity(-0.6 * MAX_SPEED)
        elif OBSTACLE_ESCAPE_MODE == "left":
            left_motor.setVelocity(1.0 * MAX_SPEED)
            right_motor.setVelocity(0.3 * MAX_SPEED)
        elif OBSTACLE_ESCAPE_MODE == "right":
            left_motor.setVelocity(0.3 * MAX_SPEED)
            right_motor.setVelocity(1.0 * MAX_SPEED)

        if now >= OBSTACLE_ESCAPE_UNTIL:
            OBSTACLE_ESCAPE = False
            OBSTACLE_ESCAPE_MODE = None
        return True

    if front > TH:
        OBSTACLE_ESCAPE = True
        OBSTACLE_ESCAPE_MODE = "front"
        OBSTACLE_ESCAPE_UNTIL = now + 1.2
        return True
    if left > TH:
        OBSTACLE_ESCAPE = True
        OBSTACLE_ESCAPE_MODE = "left"
        OBSTACLE_ESCAPE_UNTIL = now + 1.0
        return True
    if right > TH:
        OBSTACLE_ESCAPE = True
        OBSTACLE_ESCAPE_MODE = "right"
        OBSTACLE_ESCAPE_UNTIL = now + 1.0
        return True

    return False

# ============================
# NLP
# ============================
AVAILABLE_COLORS = ["red", "yellow", "green", "blue"]
AVAILABLE_SHAPES = ["ball", "cylinder", "cone"]

def parse_command(text):
    """
    Parse command text and extract color and shape.
    Returns (color, shape) or (None, None) if invalid.
    """
    if text is None:
        return None, None

    text = text.lower()

    detected_color = None
    detected_shape = None

    # find color
    for color in AVAILABLE_COLORS:
        if color in text:
            detected_color = color
            break

    # find shape
    for shape in AVAILABLE_SHAPES:
        if shape in text:
            detected_shape = shape
            break

    return detected_color, detected_shape


def load_commands(path):
    try:
        with open(path, "r") as f:
            commands = [line.strip() for line in f if line.strip()]
            return commands if commands else ["find the red ball"]
    except:
        return ["find the red ball"]


# ============================
# LOAD COMMANDS
# ============================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
COMMAND_LIST = load_commands(os.path.join(SCRIPT_DIR, "commands.txt"))

print("\n=== COMMAND LIST ===")
for i, cmd in enumerate(COMMAND_LIST):
    print(f"[{i}] {cmd}")


def advance_command():
    global CURRENT_CMD_INDEX, CURRENT_COLOR, CURRENT_SHAPE
    global SHAPE_LOCKED, rejected_cx, rejected_area, reject_until
    global SEARCH_MODE, SEARCH_UNTIL
    global rotation_count, FORWARD_SEARCH

    print("---------X----------")
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    CURRENT_CMD_INDEX += 1
    if CURRENT_CMD_INDEX >= len(COMMAND_LIST):
        print("===== ALL COMMANDS COMPLETED =====")
        return False

    command_text = COMMAND_LIST[CURRENT_CMD_INDEX]
    CURRENT_COLOR, CURRENT_SHAPE = parse_command(command_text)

    if CURRENT_COLOR is None or CURRENT_SHAPE is None:
        print("[ERROR] Invalid command:", command_text)
        print("Command must contain a valid color and shape. Going to skip it")
        time.sleep(3.0)
        return advance_command()

    print(f"\n[NEXT] Target: {CURRENT_COLOR} {CURRENT_SHAPE}")

    # reset state
    SHAPE_LOCKED = False
    rejected_cx = None
    rejected_area = None
    reject_until = 0.0
    SEARCH_MODE = False
    SEARCH_UNTIL = 0.0
    rotation_count = 0
    FORWARD_SEARCH = False

    return True
    
#============================
# CURRENT COMMAND
# ============================
CURRENT_CMD_INDEX = 0
CURRENT_COLOR, CURRENT_SHAPE = parse_command(COMMAND_LIST[0])

if CURRENT_COLOR is None or CURRENT_SHAPE is None:
    print("[ERROR] Invalid command:", COMMAND_LIST[0])
    print("Command must contain a valid color and shape. Going to skip it")
    time.sleep(3.0)
    advance_command()

print(f"\n[START] Target: {CURRENT_COLOR} {CURRENT_SHAPE}")



# ============================
# VISION
# ============================
def get_bgr():
    img = np.frombuffer(camera.getImage(), np.uint8).reshape((HEIGHT, WIDTH, 4))
    return img[:, :, :3]

def color_mask(hsv, color):
    if color == "red":
        return cv2.inRange(hsv,(0,120,70),(10,255,255)) | cv2.inRange(hsv,(170,120,70),(180,255,255))
    if color == "blue":
        return cv2.inRange(hsv,(100,160,80),(130,255,255))
    if color == "green":
        return cv2.inRange(hsv,(40,70,70),(85,255,255))
    if color == "yellow":
        return cv2.inRange(hsv,(20,100,100),(35,255,255))
    return None

def detect_color(color):
    hsv = cv2.cvtColor(get_bgr(), cv2.COLOR_BGR2HSV)
    mask = color_mask(hsv, color)
    if mask is None:
        return None, None, 0

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, 1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for cnt in contours:
        x,y,w,h = cv2.boundingRect(cnt)
        return cnt, x + w/2, w*h

    return None, None, 0

def classify_shape(cnt):
    peri = cv2.arcLength(cnt, True)
    if peri == 0:
        return "ball"
    area = cv2.contourArea(cnt)
    circularity = 4 * math.pi * area / (peri * peri)
    x,y,w,h = cv2.boundingRect(cnt)
    ar = w / float(h)
    if circularity > 0.78:
        return "ball"
    if ar < 0.7 or ar > 1.3:
        return "cylinder"
    return "cone"

# ============================
# MOTION
# ============================
def search():
    left_motor.setVelocity(0.20 * MAX_SPEED)
    right_motor.setVelocity(-0.20 * MAX_SPEED)

def approach(cx):
    if cx is None:
        return
    err = cx - FRAME_CENTER
    t = 0.4 * MAX_SPEED
    if abs(err) > CENTER_TOLERANCE:
        left_motor.setVelocity(-t if err < 0 else t)
        right_motor.setVelocity(t if err < 0 else -t)
    else:
        left_motor.setVelocity(0.8 * MAX_SPEED)
        right_motor.setVelocity(0.8 * MAX_SPEED)

# ============================
# MAIN LOOP
# ============================
color_announced = False

while robot.step(timestep) != -1:

    if SEARCH_MODE:
        search()
        if robot.getTime() >= SEARCH_UNTIL:
            SEARCH_MODE = False
        continue

    if avoid_obstacles():
        continue

    cnt, cx, area = detect_color(CURRENT_COLOR)

    # ---------- COLOR NOT FOUND ----------
    if cnt is None:
        now = robot.getTime()

        if FORWARD_SEARCH:
            left_motor.setVelocity(0.6 * MAX_SPEED)
            right_motor.setVelocity(0.6 * MAX_SPEED)
            if now >= FORWARD_UNTIL:
                FORWARD_SEARCH = False
                rotation_start_time = now
            continue

        search()

        if now - rotation_start_time >= ROTATION_TIME:
            rotation_count += 1
            rotation_start_time = now

        if rotation_count >= MAX_ROTATIONS:
            FORWARD_SEARCH = True
            FORWARD_UNTIL = now + FORWARD_TIME
            rotation_count = 0

        continue

    # ---------- COLOR FOUND ----------
    FORWARD_SEARCH = False
    rotation_count = 0

    if not color_announced:
        color_announced = True

    approach(cx)

    if area >= SHAPE_CHECK_AREA[CURRENT_SHAPE]:
        shape = classify_shape(cnt)

        if not SHAPE_LOCKED and shape != CURRENT_SHAPE:
            SEARCH_MODE = True
            SEARCH_UNTIL = robot.getTime() + 2.5
            continue

        if not SHAPE_LOCKED:
            print(f"Validated: {CURRENT_COLOR} {CURRENT_SHAPE}")
            SHAPE_LOCKED = True

        front = max(ps[0].getValue(), ps[7].getValue())
        if area >= STOP_AREA[CURRENT_SHAPE] or front >= FRONT_STOP[CURRENT_SHAPE]:
            print(f"Command Completed: {CURRENT_COLOR} {CURRENT_SHAPE}")
            stop_robot()
            time.sleep(2.0)
            color_announced = False
            SHAPE_LOCKED = False
            SEARCH_MODE= True
            SEARCH_UNTILL= robot.getTime() +3.0
            
            if not advance_command():
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
                break

print("CONTROLLER FINISHED SUCCESSFULLY")

