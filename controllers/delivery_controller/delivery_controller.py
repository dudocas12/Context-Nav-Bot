import math
import tkinter as tk
from tkinter import simpledialog
from navigation import Driver
import llm_brain

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
CURR_TURN_DIRECTION = None # "sticky" direction memory

def safe_min(region):
    """Safely gets minimum distance from a list, ignoring Infinity."""
    valid = [r for r in region if not math.isinf(r) and r > 0.0]
    return min(valid) if valid else 10.0

def best_turn_direction(left_dist, right_dist, heading_error=0):
    """Decides which way to turn based on open space + goal direction."""
    global CURR_TURN_DIRECTION
    
    SAFE_SIDE = 1.5 # Increased side safety margin
    
    # 1. Stick to current decision if we are already turning
    if CURR_TURN_DIRECTION is not None:
        return CURR_TURN_DIRECTION

    # 2. If both sides are wide open, turn towards the goal
    if left_dist > SAFE_SIDE and right_dist > SAFE_SIDE:
        # If heading error is positive (goal is left), go left
        CURR_TURN_DIRECTION = "left" if heading_error > 0 else "right"
        return CURR_TURN_DIRECTION

    # 3. Otherwise, go towards the empty space
    if left_dist > right_dist:
        CURR_TURN_DIRECTION = "left"
    else:
        CURR_TURN_DIRECTION = "right"
        
    return CURR_TURN_DIRECTION

# ==============================================================================
# 🚀 MISSION SETUP
# ==============================================================================
bot = Driver()
print("🚀 SYSTEM STARTING (REACTIVE MODE)...")

user_request = get_user_command_popup()

if user_request:
    print(f"📩 User Request: {user_request}")
    decision_dict = llm_brain.decide_destination(user_request)
    zone_name = list(decision_dict.keys())[0]
    destination = decision_dict[zone_name]
else:
    print("❌ No input. Going Home.")
    zone_name = "residential"
    destination = (0, 0) #replace later

TARGETS = [destination]
target_index = 0

# CONFIG UPDATED FOR SAFETY
SAFE_DISTANCE = 2.5      # Start avoiding from far away
CRITICAL_DISTANCE = 1.1  # Spin in place if closer than this
FORWARD_SPEED = 6.0
recovery_timer = 0
is_recovering = False

print(f"🏁 STARTING MISSION: Going to {zone_name.upper()} {destination}")

# ==============================================================================
# 🎮 MAIN LOOP
# ==============================================================================
while bot.step() != -1:
    # 1. SENSORS
    curr_x, curr_y, curr_heading = bot.get_pose()
    lidar_data = bot.get_lidar_scan()
    is_bumped = bot.check_bumpers()
    
    # 2. MISSION CHECK
    if target_index >= len(TARGETS):
        print(f"✅ ARRIVED AT {zone_name.upper()}. MISSION COMPLETE.")
        bot.stop()
        break

    t_x, t_y = TARGETS[target_index]
    dx = t_x - curr_x
    dy = t_y - curr_y
    dist = math.sqrt(dx*dx + dy*dy)
    
    # Calculate Heading Error
    target_rad = math.atan2(dy, dx)
    target_deg = math.degrees(target_rad)
    heading_error = target_deg - curr_heading
    while heading_error > 180: heading_error -= 360
    while heading_error < -180: heading_error += 360

    # 3. MISSION UPDATE
    if dist < 0.5: # 50cm tolerance
        print(f"🎉 Reached Target {target_index}!")
        target_index += 1
        continue

    # ==================================================
    # 🛡️ PRIORITY 1: BUMPER RECOVERY (The "Oh No" Reflex)
    # ==================================================
    if is_bumped and not is_recovering:
        print("💥 CRASH! Backing up.")
        is_recovering = True
        recovery_timer = 40

    if is_recovering:
        if recovery_timer > 0:
            bot.set_speed(-3.0, 0) # Back up fast
            recovery_timer -= 1
        else:
            is_recovering = False
            # Force reset sticky direction to re-evaluate
            CURR_TURN_DIRECTION = None
        continue

    # ==================================================
    # 🧠 PRIORITY 2: REACTIVE NAVIGATION
    # ==================================================
    
    # A. Process Lidar Regions
    n = len(lidar_data)
    # Use center 80% of data to avoid seeing own wheels/frame
    reduced = lidar_data[int(n*0.1) : int(n*0.9)] 
    m = len(reduced)
    
    left_region = reduced[:m // 3]
    front_region = reduced[m // 3: 2 * m // 3]
    right_region = reduced[2 * m // 3:]

    min_left = safe_min(left_region)
    min_front = safe_min(front_region)
    min_right = safe_min(right_region)

    # Defaults
    linear = 0.0
    angular = 0.0
    
    # B. Decision Tree
    if min_front < CRITICAL_DISTANCE:
        # CRITICAL: Spin in place
        CURR_TURN_DIRECTION = best_turn_direction(min_left, min_right, heading_error)
        linear = 0.0
        # Spin fast (3.0 rad/s)
        angular = 3.0 if CURR_TURN_DIRECTION == "left" else -3.0
        
    elif min_front < SAFE_DISTANCE:
        # WARNING: Drive forward but turn away
        CURR_TURN_DIRECTION = best_turn_direction(min_left, min_right, heading_error)
        
        # Slow down drastically as we get closer (0.2 factor minimum)
        # Formula ensures we drop speed fast when entering the 2.5m zone
        factor = max((min_front - CRITICAL_DISTANCE) / (SAFE_DISTANCE - CRITICAL_DISTANCE), 0.2)
        linear = FORWARD_SPEED * factor
        
        # Turn sharper as we get closer/slower
        turn_strength = 2.5 * (1.1 - factor) 
        angular = turn_strength if CURR_TURN_DIRECTION == "left" else -turn_strength
        
    else:
        # CLEAR: Drive to Goal (P-Controller)
        # Reset sticky direction since we are safe
        CURR_TURN_DIRECTION = None 
        
        linear = FORWARD_SPEED
        # Gentle heading correction
        angular = max(min(heading_error * 0.05, 2.0), -2.0)

    # 4. ACTUATE
    bot.set_speed(linear, angular)