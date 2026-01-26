import math
import tkinter as tk
from tkinter import simpledialog
from navigation import Driver
import llm_brain  # Imports your brain file

# ==============================================================================
# 🖥️ GUI POPUP FUNCTION
# ==============================================================================
def get_user_command_popup():
    """
    Opens a small window ON TOP of Webots to ask for instructions.
    This blocks the simulation until the user replies.
    """
    root = tk.Tk()
    root.withdraw() # Hide the main messy window
    
    # Make sure it appears on top of the 3D view
    root.attributes('-topmost', True) 
    
    user_input = simpledialog.askstring(
        "Robot Interface", 
        "Where should I go?\n(e.g., 'I am hungry', 'Go to the park')"
    )
    
    root.destroy()
    return user_input

# ==============================================================================
# 🚀 MISSION SETUP
# ==============================================================================
bot = Driver()
print("🚀 SYSTEM STARTING...")

# 1. POPUP THE WINDOW (Simulation pauses here)
user_request = get_user_command_popup()

# 2. ASK THE BRAIN
if user_request:
    print(f"📩 User Request: {user_request}")
    
    # NEW: The brain returns a dict like {'park': (-1.0, -1.0)}
    decision_dict = llm_brain.decide_destination(user_request)
    
    # Extract the name and coords
    zone_name = list(decision_dict.keys())[0]
    destination = decision_dict[zone_name]
else:
    print("❌ No input. Going Home.")
    zone_name = "residential"
    destination = (85.2, -5.14)

TARGETS = [destination]

# Warmup
for _ in range(20): bot.step()

target_index = 0
state = "CALCULATE"
target_heading = 0.0
evade_timer = 0 
recovery_timer = 0
CRUISE_SPEED = 8.0

# ==============================================================================
# 🎮 MAIN LOOP (Standard Logic)
# ==============================================================================
while bot.step() != -1:
    curr_x, curr_y, curr_heading = bot.get_pose()
    lidar_data = bot.get_lidar_scan()
    is_bumped = bot.check_bumpers() 
    
    front_sector = lidar_data[280:380] 
    min_front_dist = min(front_sector) if len(front_sector) > 0 else 5.0
    obstacle_detected = min_front_dist < 0.8
    
    if target_index >= len(TARGETS):
        print(f"✅ ARRIVED AT {zone_name.upper()}. MISSION COMPLETE.")
        bot.stop()
        break
    
    t_x, t_y = TARGETS[target_index]
    dx = t_x - curr_x
    dy = t_y - curr_y
    dist = math.sqrt(dx*dx + dy*dy)
    
    # --- SAFETY LOGIC ---
    if is_bumped and state != "RECOVERY":
        state = "RECOVERY"; recovery_timer = 60
    
    if state == "RECOVERY":
        if recovery_timer > 30: bot.set_speed(-2.0, 0)
        elif recovery_timer > 0: bot.set_speed(0, 3.0)
        else: state = "EVADE"; evade_timer = 20
        recovery_timer -= 1
        continue

    if obstacle_detected and state == "DRIVE": state = "AVOID"

    if state == "AVOID":
        if min_front_dist > 1.2: state = "EVADE"; evade_timer = 50
        else: bot.set_speed(0, 2.0)
            
    elif state == "EVADE":
        if evade_timer > 0:
            if min_front_dist < 0.4: state = "AVOID"
            else: bot.set_speed(3.0, 0); evade_timer -= 1
        else: state = "CALCULATE"

    elif state == "CALCULATE":
        if dist < 0.20: target_index += 1; continue
        target_rad = math.atan2(dy, dx)
        target_heading = math.degrees(target_rad)
        state = "TURN"

    elif state == "TURN":
        error = target_heading - curr_heading
        while error > 180: error -= 360
        while error < -180: error += 360
        if abs(error) < 5.0: state = "DRIVE"; continue
        bot.set_speed(0, max(min(error * 0.05, 3.0), -3.0))

    elif state == "DRIVE":
        if dist < 0.20: target_index += 1; state = "CALCULATE"; continue
        error = target_heading - curr_heading
        while error > 180: error -= 360
        while error < -180: error += 360
        speed = CRUISE_SPEED if dist > 1.0 else max(1.0, dist * 3.0)
        bot.set_speed(speed, error * 0.1)