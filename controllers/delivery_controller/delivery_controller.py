import math
from navigation import Driver

# ==============================================================================
# 🎯 MISSION: Triangle Path (With 2-Stage Crash Recovery)
# ==============================================================================
TARGETS = [
    (1.5, 1.5),    # Target 1 (Top Right)
    (-1.0, -1.0),  # Target 2 (Bottom Left)
    (1.5, -0.5)    # Target 3 (Bottom Right)
]
CRUISE_SPEED = 4.0

bot = Driver()
print("🚀 CONTROLLER STARTED: Priority Safety Mode")

# Warmup
for _ in range(20): bot.step()

target_index = 0
state = "CALCULATE"
target_heading = 0.0

# Timers
evade_timer = 0 
recovery_timer = 0

while bot.step() != -1:
    # 1. READ SENSORS
    curr_x, curr_y, curr_heading = bot.get_pose()
    lidar_data = bot.get_lidar_scan()
    is_bumped = bot.check_bumpers() 
    
    # 2. SENSOR PROCESSING
    front_sector = lidar_data[280:380] 
    min_front_dist = min(front_sector) if len(front_sector) > 0 else 5.0
    # Lidar Obstacle? (Only trusts this if bumper is NOT hit)
    obstacle_detected = min_front_dist < 0.8
    
    # 3. MISSION CHECK
    if target_index >= len(TARGETS):
        print("✅ ALL TARGETS REACHED! MISSION COMPLETE.")
        bot.stop()
        continue
    
    t_x, t_y = TARGETS[target_index]
    dx = t_x - curr_x
    dy = t_y - curr_y
    dist = math.sqrt(dx*dx + dy*dy)
    
    # ==================================================
    # 🧠 STATE MACHINE
    # ==================================================
    
    # PRIORITY 0: CRASH DETECTED (The "Ouch" Reflex)
    # If bumper hits, we FORCE Recovery, no matter what.
    if is_bumped and state != "RECOVERY":
        print("💥 CRASH! Bumper Hit. Initiating 2-Stage Recovery.")
        state = "RECOVERY"
        # We set a longer timer: 
        # Steps 60-30: Back up
        # Steps 30-0:  Turn Blindly
        recovery_timer = 60 

    # STATE: RECOVERY (The Fix for the Loop)
    if state == "RECOVERY":
        if recovery_timer > 30:
            # PHASE A: Back up significantly
            bot.set_speed(-2.0, 0)
        elif recovery_timer > 0:
            # PHASE B: Turn Left BLINDLY (Ignore Lidar)
            # We spin so we are forced to face a new direction
            bot.set_speed(0, 3.0) 
        else:
            print("🔄 Recovery Done. Resume Navigation.")
            # We switch to EVADE to drive forward away from the spot
            state = "EVADE"
            evade_timer = 20 # Short burst forward
            
        recovery_timer -= 1
        continue # SKIP all other logic while recovering

    # PRIORITY 1: LIDAR AVOIDANCE (Only if not crashing)
    if obstacle_detected and state == "DRIVE":
        print(f"⚠️ OBSTACLE ({min_front_dist:.2f}m)! Init Avoidance.")
        state = "AVOID"

    # STATE: AVOID (Lidar-based)
    if state == "AVOID":
        # If Lidar says clear, we trust it (unless we just crashed, handled above)
        if min_front_dist > 1.2: 
            print("✅ Path Clear. Starting Evasion...")
            state = "EVADE"
            evade_timer = 50 
        else:
            bot.set_speed(0, 2.0) # Turn Left
            
    # STATE: EVADE
    elif state == "EVADE":
        if evade_timer > 0:
            if min_front_dist < 0.4: # Safety stop
                state = "AVOID"
            else:
                bot.set_speed(3.0, 0)
                evade_timer -= 1
        else:
            state = "CALCULATE"

    # STATE: CALCULATE
    elif state == "CALCULATE":
        if dist < 0.20:
            target_index += 1
            continue
        target_rad = math.atan2(dy, dx)
        target_heading = math.degrees(target_rad)
        state = "TURN"

    # STATE: TURN
    elif state == "TURN":
        error = target_heading - curr_heading
        while error > 180: error -= 360
        while error < -180: error += 360
        
        if abs(error) < 5.0:
            state = "DRIVE"
            continue
        bot.set_speed(0, max(min(error * 0.05, 3.0), -3.0))

    # STATE: DRIVE
    elif state == "DRIVE":
        if dist < 0.20:
            print(f"🎉 Reached Target {target_index+1}")
            bot.stop()
            target_index += 1
            state = "CALCULATE"
            continue
            
        error = target_heading - curr_heading
        while error > 180: error -= 360
        while error < -180: error += 360
        
        speed = CRUISE_SPEED if dist > 1.0 else max(1.0, dist * 3.0)
        bot.set_speed(speed, error * 0.1)