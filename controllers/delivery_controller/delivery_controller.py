import math
import time
import tkinter as tk
from tkinter import simpledialog
from navigation import Driver
import llm_brain
from vision_brain import RobotVision  

# ==============================================================================
# 🛠️ HELPER: GUI POPUP
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
# 🧠 MODULE: NAVIGATION MATH
# ==============================================================================
CURR_TURN_DIRECTION = None 

def safe_min(region):
    valid = [r for r in region if not math.isinf(r) and r > 0.0]
    return min(valid) if valid else 10.0

def best_turn_direction(left_dist, right_dist, heading_error=0):
    global CURR_TURN_DIRECTION
    SAFE_SIDE = 1.5 
    
    if CURR_TURN_DIRECTION is not None:
        return CURR_TURN_DIRECTION

    if left_dist > SAFE_SIDE and right_dist > SAFE_SIDE:
        # If both sides open, turn towards target
        CURR_TURN_DIRECTION = "left" if heading_error > 0 else "right"
        return CURR_TURN_DIRECTION

    if left_dist > right_dist:
        CURR_TURN_DIRECTION = "left"
    else:
        CURR_TURN_DIRECTION = "right"
    return CURR_TURN_DIRECTION

# ==============================================================================
# 🎮 CONTROLLER CLASS (STATE MANAGER)
# ==============================================================================
class DeliveryController:
    def __init__(self):
        self.bot = Driver()
        self.vision = RobotVision()
        
        # Config
        self.SAFE_DISTANCE = 2.5        
        self.CRITICAL_DISTANCE = 1.1    
        self.MAX_SPEED = 6.0
        
        # State: Mission
        self.targets = []
        self.target_index = 0
        self.zone_name = "Unknown"
        
        # State: Recovery
        self.recovery_timer = 0
        self.is_recovering = False
        self.is_stuck = False
        self.stuck_escape_timer = 0
        self.last_watchdog_time = time.time()
        self.last_watchdog_pos = (0, 0)
        
        # Debug
        self.current_phase_str = ""
        
        self.vision_step_counter = 0 # Counter for 2 fps limit

    def log_phase(self, phase):
        if phase != self.current_phase_str:
            print(f"🔄 PHASE: {phase}")
            self.current_phase_str = phase

    def setup_mission(self):
        print("🚀 SYSTEM STARTING (MODULAR MODE)...")
        user_request = get_user_command_popup()

        if user_request:
            print(f"📩 User Request: {user_request}")
            decision_dict = llm_brain.decide_destination(user_request)
            self.zone_name = list(decision_dict.keys())[0]
            destination = decision_dict[self.zone_name]
        else:
            print("❌ No input. Going Home.")
            self.zone_name = "residential"
            destination = (0, 0)

        self.targets = [destination]
        print(f"🏁 STARTING MISSION: Going to {self.zone_name.upper()} {destination}")

    # --- SUB-MODULE: WATCHDOG ---
    def check_watchdog(self, curr_x, curr_y):
        if time.time() - self.last_watchdog_time > 4.0:
            dist = math.sqrt((curr_x - self.last_watchdog_pos[0])**2 + 
                             (curr_y - self.last_watchdog_pos[1])**2)
            if dist < 0.5 and not self.is_stuck and not self.is_recovering:
                self.is_stuck = True
                self.stuck_escape_timer = 50 
            self.last_watchdog_time = time.time()
            self.last_watchdog_pos = (curr_x, curr_y)

    # --- SUB-MODULE: RECOVERY ---
    def run_recovery_logic(self, is_bumped):
        # 1. Bumper Impact
        if is_bumped and not self.is_recovering:
            print("💥 CRASH! Backing up.")
            self.is_recovering = True
            self.recovery_timer = 40

        # 2. Crash Recovery Execution
        if self.is_recovering:
            self.log_phase("💥 CRASH RECOVERY")
            if self.recovery_timer > 0:
                self.bot.set_speed(-3.0, 0) 
                self.recovery_timer -= 1
            else:
                self.is_recovering = False
                global CURR_TURN_DIRECTION
                CURR_TURN_DIRECTION = None
            return True # Override active

        # 3. Stuck Recovery Execution
        if self.is_stuck:
            self.log_phase("🐕 WATCHDOG ESCAPE")
            if self.stuck_escape_timer > 0:
                self.bot.set_speed(-3.0, 5.0) 
                self.stuck_escape_timer -= 1
            else:
                self.is_stuck = False
                CURR_TURN_DIRECTION = None 
            return True # Override active

        return False # No recovery needed

    # --- SUB-MODULE: NAVIGATION ---
    def run_navigation_logic(self, lidar_data, is_ground_safe, dist, heading_error):
        n = len(lidar_data)
        reduced = lidar_data[int(n*0.1) : int(n*0.9)] 
        m = len(reduced)
        
        min_left = safe_min(reduced[:m // 3])
        min_front = safe_min(reduced[m // 3: 2 * m // 3])
        min_right = safe_min(reduced[2 * m // 3:])

        # VISION INJECTION: If ground unsafe, fake a close obstacle
        if not is_ground_safe:
            print(f"⚠️ VISION: ROAD DETECTED! (Lidar: {min_front:.2f}m -> FORCED: 0.2m)")
            min_front = 0.2 

        linear = 0.0
        angular = 0.0
        global CURR_TURN_DIRECTION
        
        if min_front < self.CRITICAL_DISTANCE:
            self.log_phase("🔄 CRITICAL AVOIDANCE (SPIN)")
            CURR_TURN_DIRECTION = best_turn_direction(min_left, min_right, heading_error)
            linear = 0.0
            angular = 3.0 if CURR_TURN_DIRECTION == "left" else -3.0
            
        elif min_front < self.SAFE_DISTANCE:
            self.log_phase("⚠️ AVOIDING OBSTACLE/ROAD")
            CURR_TURN_DIRECTION = best_turn_direction(min_left, min_right, heading_error)
            factor = max((min_front - self.CRITICAL_DISTANCE) / (self.SAFE_DISTANCE - self.CRITICAL_DISTANCE), 0.2)
            linear = self.MAX_SPEED * factor
            turn_strength = 2.5 * (1.1 - factor) 
            angular = turn_strength if CURR_TURN_DIRECTION == "left" else -turn_strength
            
        else:
            self.log_phase("🟢 CRUISING")
            CURR_TURN_DIRECTION = None 
            if dist < 3.0:
                linear = max(2.0, self.MAX_SPEED * (dist / 3.0))
            else:
                linear = self.MAX_SPEED
            angular = max(min(heading_error * 0.05, 2.0), -2.0)

        self.bot.set_speed(linear, angular)

    # --- MAIN LOOP ---
    def run(self):
        self.setup_mission()

        while self.bot.step() != -1:
            # 1. READ SENSORS
            curr_x, curr_y, curr_heading = self.bot.get_pose()
            lidar_data = self.bot.get_lidar_scan()
            is_bumped = self.bot.check_bumpers()
            ground_img = self.bot.get_ground_image()
            front_img = self.bot.get_front_image()
            self.vision_step_counter += 1
            
            # 2. Check for Yellow Lines first (Fast)
            is_crosswalk = self.vision.detect_crosswalk(ground_img, 64, 64)
            
            # 3. Run YOLO ONLY if:
            #    a) We are on a crosswalk
            #    b) It is time for a refresh (Every ~15 steps = 2 FPS)
            if is_crosswalk and (self.vision_step_counter % 15 == 0):
                front_img = self.bot.get_front_image()
                self.vision.scan_for_traffic_lights(front_img, 416, 416)
            elif not is_crosswalk:
                # Optional: Close the window if not needed, or just leave it static
                # cv2.destroyAllWindows() 
                pass

            # 2. CHECK MISSION
            if self.target_index >= len(self.targets):
                print(f"✅ ARRIVED AT {self.zone_name.upper()}. MISSION COMPLETE.")
                self.bot.stop()
                break

            t_x, t_y = self.targets[self.target_index]
            dx = t_x - curr_x
            dy = t_y - curr_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            target_rad = math.atan2(dy, dx)
            target_deg = math.degrees(target_rad)
            heading_error = target_deg - curr_heading
            while heading_error > 180: heading_error -= 360
            while heading_error < -180: heading_error += 360

            if dist < 0.5: 
                print(f"🎉 Reached Target {self.target_index}!")
                self.target_index += 1
                continue

            # 3. RUN LOGIC MODULES
            self.check_watchdog(curr_x, curr_y)
            
            # Check Ground Safety (Vision Brain)
            is_ground_safe = self.vision.check_ground_safety(ground_img, 64, 64)

            # Priority: Recovery -> Navigation
            if not self.run_recovery_logic(is_bumped):
                self.run_navigation_logic(lidar_data, is_ground_safe, dist, heading_error)

# ==============================================================================
# 🚀 ENTRY POINT
# ==============================================================================
if __name__ == "__main__":
    controller = DeliveryController()
    controller.run()