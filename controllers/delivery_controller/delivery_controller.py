import math
from navigation import DeliveryRobot

bot = DeliveryRobot()

# SETTINGS
STOP_DIST = 0.6
CLEAR_DIST = 1.0
CRITICAL_DIST = 0.4

# STATE MEMORY
is_stuck = False
stuck_direction = 0
prev_dist = 10.0

# STUCK DETECTION (GPS-Based)
stuck_timer = 0
last_position = (0, 0)

print("🤖 Robot Logic: Lidar + GPS Stuck Detection")

while bot.step() != -1:
    lidar_data = bot.get_lidar_data()
    x, y, theta = bot.get_pose()
    
    if not lidar_data: continue

    # 1. PROCESS SENSORS
    total_rays = len(lidar_data)
    mid_point = total_rays // 2
    
    right_part = lidar_data[mid_point - 120 : mid_point - 20]
    center_part = lidar_data[mid_point - 20 : mid_point + 20]
    left_part = lidar_data[mid_point + 20 : mid_point + 120]

    def get_dist(rays):
        valid = [d for d in rays if d != float('inf')]
        return min(valid) if valid else 10.0

    dist_right = get_dist(right_part)
    dist_front = get_dist(center_part)
    dist_left = get_dist(left_part)

    # Blind Spot Fix
    if dist_front == 10.0 and prev_dist < 0.5:
        dist_front = 0.05 
    prev_dist = dist_front

    # 2. GPS STUCK CHECK (The "Invisible Wall" Fix)
    # Calculate how far we moved since last check
    dx = x - last_position[0]
    dy = y - last_position[1]
    dist_moved = math.sqrt(dx*dx + dy*dy)
    
    # If we are trying to drive (not stuck mode), but not moving...
    if not is_stuck and dist_moved < 0.002: # 2mm movement threshold
        stuck_timer += 1
    else:
        stuck_timer = 0
        last_position = (x, y)

    # If we haven't moved for 20 frames (~0.6 seconds), we are hitting something!
    if stuck_timer > 20:
        print("🛑 GPS says we are stuck! (Invisible Obstacle). Forcing Reverse.")
        is_stuck = True
        stuck_direction = 1 # Force a turn
        stuck_timer = 0

    # 3. LOGIC
    if not is_stuck and dist_front < STOP_DIST:
        is_stuck = True
        if dist_left > dist_right:
            stuck_direction = 1  
            print(f"🛑 Blocked! Locking turn LEFT.")
        else:
            stuck_direction = -1 
            print(f"🛑 Blocked! Locking turn RIGHT.")

    if is_stuck and dist_front > CLEAR_DIST:
        is_stuck = False
        stuck_direction = 0
        print("✅ Path Found. Resuming Drive.")

    # 4. EXECUTE
    if is_stuck:
        if dist_front < CRITICAL_DIST:
             print(f"⚠️ Too Close ({dist_front:.2f}m). Reversing...")
             bot.set_speed(-0.3, 0.0)
        else:
            if stuck_direction == 1:
                bot.set_speed(0.0, 1.5) 
            else:
                bot.set_speed(0.0, -1.5) 
    else:
        speed = 1.0 if dist_front > 1.5 else 0.5
        turn = 0.0
        if dist_left < 0.8: turn = -0.4
        if dist_right < 0.8: turn = 0.4
        bot.set_speed(speed, turn)

    print(f"Front: {dist_front:.2f}m | GPS Move: {dist_moved:.4f}")