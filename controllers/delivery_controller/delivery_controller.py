import math
from navigation import Driver

# ==============================================================================
# 🎯 MISSION: Triangle Path
# ==============================================================================
# I reduced these slightly just in case (2,2) is still off-limits. 
# Feel free to change them back if your map is big enough!
TARGETS = [
    (1.5, 1.5),    # Target 1 (Top Right)
    (-1.0, -1.0),  # Target 2 (Bottom Left)
    (1.5, -0.5)    # Target 3 (Bottom Right)
]
CRUISE_SPEED = 4.0

bot = Driver()
print("🚀 CONTROLLER STARTED")

# Warmup sensors
for _ in range(20): bot.step()

target_index = 0
state = "CALCULATE"
target_heading = 0.0

while bot.step() != -1:
    # 1. READ SENSORS
    curr_x, curr_y, curr_heading = bot.get_pose()
    
    # 2. CHECK IF MISSION COMPLETE
    if target_index >= len(TARGETS):
        print("✅ ALL TARGETS REACHED! MISSION COMPLETE.")
        bot.stop()
        continue
        
    # 3. GET CURRENT TARGET
    t_x, t_y = TARGETS[target_index]
    dx = t_x - curr_x
    dy = t_y - curr_y
    dist = math.sqrt(dx*dx + dy*dy)
    
    # ==================================================
    # 🧠 STATE MACHINE
    # ==================================================
    
    if state == "CALCULATE":
        # Check if we spawned ON the target (Safety check)
        if dist < 0.20:
            print(f"🎉 Already at ({t_x}, {t_y})")
            target_index += 1
            continue
        
        # Calculate Angle
        target_rad = math.atan2(dy, dx)
        target_heading = math.degrees(target_rad)
        print(f"📍 New Target: ({t_x}, {t_y}) | Dist: {dist:.2f}m | Heading: {target_heading:.1f}°")
        state = "TURN"

    elif state == "TURN":
        error = target_heading - curr_heading
        while error > 180: error -= 360
        while error < -180: error += 360
        
        # If aligned, switch to driving
        if abs(error) < 2.0:
            bot.stop() # Brief stop to settle
            state = "DRIVE"
            continue
            
        # P-Controller for Turn
        turn_speed = error * 0.05
        bot.set_speed(0, turn_speed)

    elif state == "DRIVE":
        # --- 1. SUCCESS CHECK ---
        if dist < 0.20:
            print(f"🎉 Reached Target {target_index+1} at ({t_x}, {t_y})")
            bot.stop()
            target_index += 1  # Increment target
            state = "CALCULATE"
            continue
            
        # --- 2. HEADING CORRECTION ---
        error = target_heading - curr_heading
        while error > 180: error -= 360
        while error < -180: error += 360
        correction = error * 0.1
        
        # --- 3. SMART BRAKING ---
        if dist > 1.0:
            speed = CRUISE_SPEED
        else:
            speed = max(1.0, dist * 3.0) # Slow down smoothly
            
        bot.set_speed(speed, correction)
        
        # Debug print (Fixed the comma typo)
        print(f"🚗 Dist: {dist:.3f}")