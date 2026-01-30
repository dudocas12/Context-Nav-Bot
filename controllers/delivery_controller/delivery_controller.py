import math
import time
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

class DeliveryController:
    def __init__(self):
        self.bot = Driver()
        self.vision = RobotVision()
        
        # Config
        self.SAFE_DISTANCE = 2.5        
        self.CRITICAL_DISTANCE = 1.1    
        self.MAX_SPEED = 6.0
        
        # Mission State
        self.targets = []
        self.target_index = 0
        self.zone_name = "Unknown"
        
        # LOGIC STATE MACHINE
        self.state = "CRUISING"  
        self.scan_timer = 0      
        self.retreat_timer = 0   
        self.commit_timer = 0    
        self.waiting_timer = 0   
        
        # Recovery
        self.recovery_timer = 0
        self.is_recovering = False
        self.is_stuck = False
        self.stuck_escape_timer = 0
        self.last_watchdog_time = time.time()
        self.last_watchdog_pos = (0, 0)
        
        self.current_phase_str = ""
        self.vision_step_counter = 0

    def log_phase(self, phase):
        if phase != self.current_phase_str:
            print(f"🔄 STATE: {phase}")
            self.current_phase_str = phase

    def setup_mission(self):
        print("🚀 SYSTEM STARTING (RIGHT SCAN MODE)...")
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

    # --- WATCHDOG & RECOVERY ---
    def check_watchdog(self, curr_x, curr_y):
        if self.state == "CRUISING" and time.time() - self.last_watchdog_time > 4.0:
            dist = math.sqrt((curr_x - self.last_watchdog_pos[0])**2 + (curr_y - self.last_watchdog_pos[1])**2)
            if dist < 0.5 and not self.is_stuck and not self.is_recovering:
                self.is_stuck = True
                self.stuck_escape_timer = 50 
            self.last_watchdog_time = time.time()
            self.last_watchdog_pos = (curr_x, curr_y)

    def run_recovery_logic(self, is_bumped):
        if is_bumped and not self.is_recovering:
            print("💥 CRASH! Backing up.")
            self.is_recovering = True
            self.recovery_timer = 40
        if self.is_recovering:
            self.log_phase("💥 CRASH RECOVERY")
            if self.recovery_timer > 0:
                self.bot.set_speed(-3.0, 0) 
                self.recovery_timer -= 1
            else:
                self.is_recovering = False
                global CURR_TURN_DIRECTION; CURR_TURN_DIRECTION = None
            return True 
        if self.is_stuck:
            self.log_phase("🐕 WATCHDOG ESCAPE")
            if self.stuck_escape_timer > 0:
                self.bot.set_speed(-3.0, 5.0) 
                self.stuck_escape_timer -= 1
            else:
                self.is_stuck = False
                CURR_TURN_DIRECTION = None 
            return True 
        return False 

    # --- NAVIGATION LOGIC ---
    def run_navigation_logic(self, lidar_data, is_ground_safe, dist, heading_error):
        n = len(lidar_data)
        reduced = lidar_data[int(n*0.1) : int(n*0.9)] 
        m = len(reduced)
        min_left = safe_min(reduced[:m // 3])
        min_front = safe_min(reduced[m // 3: 2 * m // 3])
        min_right = safe_min(reduced[2 * m // 3:])

        if not is_ground_safe:
            print(f"⚠️ VISION: ROAD DETECTED! (Lidar: {min_front:.2f}m -> FORCED: 0.2m)")
            min_front = 0.2 

        linear = 0.0; angular = 0.0
        global CURR_TURN_DIRECTION
        
        if min_front < self.CRITICAL_DISTANCE:
            CURR_TURN_DIRECTION = best_turn_direction(min_left, min_right, heading_error)
            linear = 0.0; angular = 3.0 if CURR_TURN_DIRECTION == "left" else -3.0
        elif min_front < self.SAFE_DISTANCE:
            CURR_TURN_DIRECTION = best_turn_direction(min_left, min_right, heading_error)
            factor = max((min_front - self.CRITICAL_DISTANCE) / (self.SAFE_DISTANCE - self.CRITICAL_DISTANCE), 0.2)
            linear = self.MAX_SPEED * factor
            turn_strength = 2.5 * (1.1 - factor) 
            angular = turn_strength if CURR_TURN_DIRECTION == "left" else -turn_strength
        else:
            CURR_TURN_DIRECTION = None 
            if dist < 3.0: linear = max(2.0, self.MAX_SPEED * (dist / 3.0))
            else: linear = self.MAX_SPEED
            angular = max(min(heading_error * 0.05, 2.0), -2.0)

        self.bot.set_speed(linear, angular)

    # --- MAIN LOOP ---
    def run(self):
        self.setup_mission()

        while self.bot.step() != -1:
            curr_x, curr_y, curr_heading = self.bot.get_pose()
            lidar_data = self.bot.get_lidar_scan()
            is_bumped = self.bot.check_bumpers()
            ground_img = self.bot.get_ground_image()
            self.vision_step_counter += 1

            # MISSION CHECK
            if self.target_index >= len(self.targets):
                print(f"✅ ARRIVED AT {self.zone_name.upper()}. MISSION COMPLETE.")
                self.bot.stop(); break

            t_x, t_y = self.targets[self.target_index]
            dx = t_x - curr_x; dy = t_y - curr_y
            dist = math.sqrt(dx*dx + dy*dy)
            target_rad = math.atan2(dy, dx)
            target_deg = math.degrees(target_rad)
            heading_error = target_deg - curr_heading
            while heading_error > 180: heading_error -= 360
            while heading_error < -180: heading_error += 360

            if dist < 0.5: 
                print(f"🎉 Reached Target {self.target_index}!"); self.target_index += 1; continue

            # RECOVERY CHECK
            self.check_watchdog(curr_x, curr_y)
            if self.run_recovery_logic(is_bumped): continue

            # ==========================================================
            # 🚦 STATE MACHINE LOGIC
            # ==========================================================
            
            # --- STATE 1: CRUISING ---
            if self.state == "CRUISING":
                self.log_phase("🟢 CRUISING")
                
                is_crosswalk = self.vision.detect_crosswalk(ground_img, 64, 64)
                
                if is_crosswalk:
                    print("🦓 CROSSWALK DETECTED - STOPPING & SCANNING")
                    self.bot.stop()
                    self.state = "SCANNING"
                    self.scan_timer = 0
                else:
                    is_ground_safe = self.vision.check_ground_safety(ground_img, 64, 64)
                    self.run_navigation_logic(lidar_data, is_ground_safe, dist, heading_error)

            # --- STATE 2: SCANNING (Rotate RIGHT to find light) ---
            elif self.state == "SCANNING":
                self.log_phase("👀 SCANNING FOR LIGHT")
                
                # CHANGED: Negative angular velocity = Rotate RIGHT
                self.bot.set_speed(0, -0.5) 
                self.scan_timer += 1
                
                if self.scan_timer > 400:
                    print("⚠️ SCAN TIMEOUT. RETREATING TO RESET...")
                    self.state = "RETREAT_TURN"
                    self.retreat_timer = 130 
                    continue

                if self.vision_step_counter % 15 == 0:
                    front_img = self.bot.get_front_image()
                    light_data = self.vision.scan_for_traffic_lights(front_img, 416, 416)
                    
                    if light_data['found']:
                        if light_data['color'] != 'unknown':
                            print(f"🚦 FOUND VALID LIGHT ({light_data['color'].upper()}) - LOCKING ON")
                            self.bot.stop()
                            self.state = "WAITING"
                            self.waiting_timer = 0
                        else:
                            print(f"⚠️ IGNORED DISTANT LIGHT (Unknown Color)")

            # --- STATE 3: RETREATING (Turn 180) ---
            elif self.state == "RETREAT_TURN":
                self.log_phase("🔙 RETREATING (TURNING 180)")
                self.bot.set_speed(0, 2.5) 
                self.retreat_timer -= 1
                if self.retreat_timer <= 0:
                    self.state = "RETREAT_DRIVE"
                    self.retreat_timer = 150 

            # --- STATE 4: RETREATING (Drive Away) ---
            elif self.state == "RETREAT_DRIVE":
                self.log_phase("🔙 RETREATING (DRIVING AWAY)")
                n = len(lidar_data)
                min_front = safe_min(lidar_data[int(n*0.3) : int(n*0.7)])
                
                if min_front < 1.0:
                    self.bot.set_speed(0, 2.0) 
                else:
                    self.bot.set_speed(4.0, 0) 
                
                self.retreat_timer -= 1
                if self.retreat_timer <= 0:
                    print("🔄 RETREAT COMPLETE - RESUMING CRUISE")
                    self.state = "CRUISING"

            # --- STATE 5: WAITING (Stare at light) ---
            elif self.state == "WAITING":
                self.log_phase("🛑 WAITING FOR GREEN")
                self.bot.set_speed(0, 0)
                self.waiting_timer += 1
                
                if self.waiting_timer > 1000:
                    print("⚠️ WAITING TIMEOUT (STUCK ON RED) - RETREATING...")
                    self.state = "RETREAT_TURN"
                    self.retreat_timer = 130
                    continue

                if self.vision_step_counter % 15 == 0:
                    front_img = self.bot.get_front_image()
                    light_data = self.vision.scan_for_traffic_lights(front_img, 416, 416)
                    
                    if light_data['found'] and light_data['color'] == 'green':
                        print("✅ GREEN LIGHT! ALIGNING TO TARGET...")
                        self.state = "ALIGNING"

            # --- STATE 6: ALIGNING (Rotate to target) ---
            elif self.state == "ALIGNING":
                self.log_phase("📐 ALIGNING TO TARGET")
                if abs(heading_error) > 5.0:
                    ang_vel = 2.0 if heading_error > 0 else -2.0
                    self.bot.set_speed(0, ang_vel)
                else:
                    print("🚀 COMMITTING TO CROSSING (1000 STEPS)")
                    self.bot.stop()
                    self.state = "COMMITTING"
                    self.commit_timer = 1000 

            # --- STATE 7: COMMITTING (Blind Run) ---
            elif self.state == "COMMITTING":
                self.log_phase(f"🚀 CROSSING COMMITMENT ({self.commit_timer})")
                is_ground_safe = True 
                self.run_navigation_logic(lidar_data, is_ground_safe, dist, heading_error)
                self.commit_timer -= 1
                if self.commit_timer <= 0:
                    print("🏁 COMMITMENT DONE - RESUMING PATROL")
                    self.state = "CRUISING"

if __name__ == "__main__":
    controller = DeliveryController()
    controller.run()