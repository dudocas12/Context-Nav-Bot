import math
import time
import random
import tkinter as tk
from tkinter import simpledialog
from navigation import Driver
import llm_brain
from vision_brain import RobotVision  

# ... (Standard Helper Functions) ...
def get_user_command_popup():
    root = tk.Tk()
    root.withdraw()
    root.attributes('-topmost', True) 
    user_input = simpledialog.askstring("Robot Interface", "Where should I go?")
    root.destroy()
    return user_input

CURR_TURN_DIRECTION = None 
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
    if CURR_TURN_DIRECTION is not None: return CURR_TURN_DIRECTION
    if left_dist > SAFE_SIDE and right_dist > SAFE_SIDE:
        CURR_TURN_DIRECTION = "left" if heading_error > 0 else "right"
        return CURR_TURN_DIRECTION
    if left_dist > right_dist: CURR_TURN_DIRECTION = "left"
    else: CURR_TURN_DIRECTION = "right"
    return CURR_TURN_DIRECTION

# ==============================================================================
# 🚀 MISSION SETUP
# ==============================================================================
bot = Driver()
vision = RobotVision() 

print("🚀 SYSTEM STARTING (FINAL COMMITMENT MODE)...")

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

SAFE_DISTANCE = 2.5       
CRITICAL_DISTANCE = 1.1   
MAX_SPEED = 6.0

# VISION & STATE VARIABLES
vision_step_counter = 0
VISION_SKIP_RATE = 5 
empty_light_data = {'found': False, 'color': 'none', 'center_x': 0.5, 'box_width': 0}
light_data = empty_light_data
crosswalk_cooldown = 0

# MEMORY VARIABLES
red_light_memory = 0  # Timer to remember Red light if it flickers

# SEARCH STATE MACHINE
search_mode = "none" # "scan_left", "scan_right", "re_center"
search_timer = 0

print(f"🏁 STARTING MISSION: Going to {zone_name.upper()} {destination}")

# ==============================================================================
# 🎮 MAIN LOOP
# ==============================================================================
while bot.step() != -1:
    vision_step_counter += 1
    curr_x, curr_y, curr_heading = bot.get_pose()
    lidar_data = bot.get_lidar_scan()
    
    # 1. VISION ACQUISITION
    ground_img = bot.get_ground_image()
    is_ground_safe = vision.check_ground_safety(ground_img, 64, 64)
    is_crosswalk = vision.detect_crosswalk(ground_img, 64, 64)
    
    # 2. SMART YOLO 
    should_run_yolo = is_crosswalk or (search_mode != "none") or (red_light_memory > 0)
    
    if should_run_yolo:
        if vision_step_counter % VISION_SKIP_RATE == 0:
            front_img = bot.get_front_image()
            light_data = vision.scan_for_traffic_lights(front_img, 416, 416)
    else:
        light_data = empty_light_data

    # Decrement Timers
    if crosswalk_cooldown > 0: crosswalk_cooldown -= 1
    if search_timer > 0: search_timer -= 1
    if red_light_memory > 0: red_light_memory -= 1

    # 3. MISSION CHECK
    if target_index >= len(TARGETS):
        print("✅ MISSION COMPLETE.")
        bot.stop()
        break

    # Navigation Calc
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
    # 🚦 & 🦓 INTELLIGENT NAVIGATION LAYER
    # ==================================================
    
    # A. RESET SEARCH IF FOUND
    if light_data['found'] and search_mode != "none":
        print_phase("👀 SIGNAL FOUND! STOPPING SEARCH.")
        search_mode = "none"
        bot.stop()

    # B. CROSSING LOGIC
    if crosswalk_cooldown > 0:
        crossing_active = True
    else:
        crossing_active = False
        
        # --- LIGHT DETECTION HANDLING ---
        if light_data['found']:
            if light_data['color'] == 'green':
                print_phase("✅ GREEN LIGHT - GOING!")
                # UPDATED: 1000 steps (~30 seconds) of Blind Commitment
                crosswalk_cooldown = 1000 
                crossing_active = True
                search_mode = "none"
                red_light_memory = 0 
                
            elif light_data['color'] == 'red':
                print_phase("🛑 RED LIGHT - WAITING")
                bot.set_speed(0, 0)
                # Remember Red light for ~1.5 seconds
                red_light_memory = 45 
                continue 
        
        # --- MEMORY HANDLING ---
        elif red_light_memory > 0:
            print_phase("🛑 (MEMORY) WAITING FOR GREEN...")
            bot.set_speed(0, 0)
            continue 

        # --- SEARCH TRIGGER ---
        elif is_crosswalk and search_mode == "none":
            print_phase("👀 NO SIGNAL - SCANNING LEFT...")
            search_mode = "scan_left"
            search_timer = 25 
            bot.set_speed(0, 0)
            continue

    # C. EXECUTE SEARCH MOVEMENT
    if search_mode != "none":
        if search_mode == "scan_left":
            bot.set_speed(0, 2.0) 
            if search_timer == 0:
                print_phase("👀 SCANNING RIGHT...")
                search_mode = "scan_right"
                search_timer = 50 

        elif search_mode == "scan_right":
            bot.set_speed(0, -2.0) 
            if search_timer == 0:
                print_phase("👀 RE-CENTERING...")
                search_mode = "re_center"
                search_timer = 25 

        elif search_mode == "re_center":
            bot.set_speed(0, 2.0) 
            if search_timer == 0:
                print_phase("⚠️ SCAN FAILED - YIELDING...")
                search_mode = "none"
        
        continue 

    # ==================================================
    # 🧠 REACTIVE NAVIGATION
    # ==================================================
    n = len(lidar_data)
    reduced = lidar_data[int(n*0.1) : int(n*0.9)] 
    m = len(reduced)
    min_left = safe_min(reduced[:m // 3])
    min_front = safe_min(reduced[m // 3: 2 * m // 3])
    min_right = safe_min(reduced[2 * m // 3:])

    # GROUND SAFETY OVERRIDE
    if not is_ground_safe and not crossing_active:
        print_phase("🚧 KEEPING ON SIDEWALK")
        min_front = 0.5 
    
    linear = 0.0
    angular = 0.0
    
    if min_front < CRITICAL_DISTANCE:
        CURR_TURN_DIRECTION = best_turn_direction(min_left, min_right, heading_error)
        linear = 0.0
        angular = 3.0 if CURR_TURN_DIRECTION == "left" else -3.0
    elif min_front < SAFE_DISTANCE:
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