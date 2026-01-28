import math
import time
import random
import tkinter as tk
from tkinter import simpledialog
from navigation import Driver
import llm_brain
from vision_brain import RobotVision  

# ==============================================================================
# 🖥️ GUI POPUP FUNCTION
# ==============================================================================
def get_user_command_popup():
    root = tk.Tk()
    root.withdraw()
    root.attributes('-topmost', True) 
    
    user_input = simpledialog.askstring(
        "Robot Interface", 
        "Where should I go?\n(e.g., 'I am hungry', 'Go to the park')"
    )
    
    root.destroy()
    return user_input

# ==============================================================================
# 🚦 REACTIVE LOGIC HELPERS
# ==============================================================================
CURR_TURN_DIRECTION = None 

# --- PHASE PRINTER ---
current_phase_str = ""
def print_phase(phase):
    global current_phase_str
    if phase != current_phase_str:
        print(f"🔄 PHASE: {phase}")
        current_phase_str = phase

def safe_min(region):
    valid = [r for r in region if not math.isinf(r) and r > 0.0]
    return min(valid) if valid else 10.0

def best_turn_direction(left_dist, right_dist, heading_error=0):
    global CURR_TURN_DIRECTION
    SAFE_SIDE = 1.5 
    
    if CURR_TURN_DIRECTION is not None:
        return CURR_TURN_DIRECTION

    if left_dist > SAFE_SIDE and right_dist > SAFE_SIDE:
        CURR_TURN_DIRECTION = "left" if heading_error > 0 else "right"
        return CURR_TURN_DIRECTION

    if left_dist > right_dist:
        CURR_TURN_DIRECTION = "left"
    else:
        CURR_TURN_DIRECTION = "right"
    return CURR_TURN_DIRECTION

# ==============================================================================
# 🚀 MISSION SETUP
# ==============================================================================
bot = Driver()
vision = RobotVision() 

print("🚀 SYSTEM STARTING (NO YOLO MODE)...")

user_request = get_user_command_popup()

if user_request:
    print(f"📩 User Request: {user_request}")
    decision_dict = llm_brain.decide_destination(user_request)
    zone_name = list(decision_dict.keys())[0]
    destination = decision_dict[zone_name]
else:
    print("❌ No input. Going Home.")
    zone_name = "residential"
    destination = (0, 0)

TARGETS = [destination]
target_index = 0

# CONFIG
SAFE_DISTANCE = 2.5       
CRITICAL_DISTANCE = 1.1   
MAX_SPEED = 6.0

# STATE
recovery_timer = 0
is_recovering = False
last_watchdog_time = time.time()
last_watchdog_pos = (0, 0)
stuck_escape_timer = 0
is_stuck = False

print(f"🏁 STARTING MISSION: Going to {zone_name.upper()} {destination}")

# ==============================================================================
# 🎮 MAIN LOOP
# ==============================================================================
while bot.step() != -1:
    # 1. SENSORS
    curr_x, curr_y, curr_heading = bot.get_pose()
    lidar_data = bot.get_lidar_scan()
    is_bumped = bot.check_bumpers()
    
    # --- VISION CHECK ---
    ground_img = bot.get_ground_image()
    is_ground_safe = vision.check_ground_safety(ground_img, 64, 64)

    # 2. MISSION CHECK
    if target_index >= len(TARGETS):
        print(f"✅ ARRIVED AT {zone_name.upper()}. MISSION COMPLETE.")
        bot.stop()
        break

    t_x, t_y = TARGETS[target_index]
    dx = t_x - curr_x
    dy = t_y - curr_y
    dist = math.sqrt(dx*dx + dy*dy)
    
    target_rad = math.atan2(dy, dx)
    target_deg = math.degrees(target_rad)
    heading_error = target_deg - curr_heading
    while heading_error > 180: heading_error -= 360
    while heading_error < -180: heading_error += 360

    if dist < 0.5: 
        print(f"🎉 Reached Target {target_index}!")
        target_index += 1
        continue

    # ==================================================
    # 🐕 WATCHDOG (STUCK DETECTOR)
    # ==================================================
    if time.time() - last_watchdog_time > 4.0:
        dist_moved = math.sqrt((curr_x - last_watchdog_pos[0])**2 + (curr_y - last_watchdog_pos[1])**2)
        if dist_moved < 0.5 and not is_stuck and not is_recovering:
            is_stuck = True
            stuck_escape_timer = 50 
        last_watchdog_time = time.time()
        last_watchdog_pos = (curr_x, curr_y)

    if is_stuck:
        print_phase("🐕 WATCHDOG ESCAPE")
        if stuck_escape_timer > 0:
            bot.set_speed(-3.0, 5.0) 
            stuck_escape_timer -= 1
        else:
            is_stuck = False
            CURR_TURN_DIRECTION = None 
        continue

    # ==================================================
    # 🛡️ PRIORITY 1: BUMPER RECOVERY
    # ==================================================
    if is_bumped and not is_recovering:
        print("💥 CRASH! Backing up.")
        is_recovering = True
        recovery_timer = 40

    if is_recovering:
        print_phase("💥 CRASH RECOVERY")
        if recovery_timer > 0:
            bot.set_speed(-3.0, 0) 
            recovery_timer -= 1
        else:
            is_recovering = False
            CURR_TURN_DIRECTION = None
        continue

    # ==================================================
    # 🧠 PRIORITY 2: REACTIVE NAVIGATION (With Vision)
    # ==================================================
    
    n = len(lidar_data)
    reduced = lidar_data[int(n*0.1) : int(n*0.9)] 
    m = len(reduced)
    
    left_region = reduced[:m // 3]
    front_region = reduced[m // 3: 2 * m // 3]
    right_region = reduced[2 * m // 3:]

    min_left = safe_min(left_region)
    min_front = safe_min(front_region)
    min_right = safe_min(right_region)

    # --- VISION INJECTION ---
    if not is_ground_safe:
        print(f"⚠️ VISION: ROAD DETECTED! (Lidar: {min_front:.2f}m -> FORCED: 0.2m)")
        min_front = 0.2 

    linear = 0.0
    angular = 0.0
    
    if min_front < CRITICAL_DISTANCE:
        print_phase("🔄 CRITICAL AVOIDANCE (SPIN)")
        CURR_TURN_DIRECTION = best_turn_direction(min_left, min_right, heading_error)
        linear = 0.0
        angular = 3.0 if CURR_TURN_DIRECTION == "left" else -3.0
        
    elif min_front < SAFE_DISTANCE:
        print_phase("⚠️ AVOIDING OBSTACLE/ROAD")
        CURR_TURN_DIRECTION = best_turn_direction(min_left, min_right, heading_error)
        factor = max((min_front - CRITICAL_DISTANCE) / (SAFE_DISTANCE - CRITICAL_DISTANCE), 0.2)
        linear = MAX_SPEED * factor
        turn_strength = 2.5 * (1.1 - factor) 
        angular = turn_strength if CURR_TURN_DIRECTION == "left" else -turn_strength
        
    else:
        print_phase("🟢 CRUISING")
        CURR_TURN_DIRECTION = None 
        if dist < 3.0:
            linear = max(2.0, MAX_SPEED * (dist / 3.0))
        else:
            linear = MAX_SPEED
        angular = max(min(heading_error * 0.05, 2.0), -2.0)

    bot.set_speed(linear, angular)