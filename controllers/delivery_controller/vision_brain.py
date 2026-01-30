import cv2
import numpy as np
from ultralytics import YOLO

class RobotVision:
    def __init__(self):
        # --- GROUND CAMERA SETTINGS ---
        self.ABSOLUTE_DARK_THRESHOLD = 25 
        self.BLUE_RATIO_THRESHOLD = 1.3
        self.SAFE_PIXEL_COUNT = 200
        
        # --- FRONT CAMERA SETTINGS ---
        print("🧠 LOADING YOLO MODEL...")
        try:
            self.model = YOLO('yolo11n.pt') 
            print("✅ YOLO Model Loaded")
        except:
            print("❌ YOLO Failed")
            self.model = None

        self.TRAFFIC_LIGHT_CLASS_ID = 9 
        
        # Traffic Light Colors (HSV)
        # UPDATED: Widened Green range to catch bright/cyan greens
        self.GREEN_LOWER = np.array([35, 40, 40])
        self.GREEN_UPPER = np.array([95, 255, 255])
        
        self.RED_LOWER1 = np.array([0, 50, 50])
        self.RED_UPPER1 = np.array([10, 255, 255])
        self.RED_LOWER2 = np.array([160, 50, 50])
        self.RED_UPPER2 = np.array([180, 255, 255])

        # Crosswalk Color (Yellow HSV)
        self.YELLOW_LOWER = np.array([20, 100, 100])
        self.YELLOW_UPPER = np.array([40, 255, 255])

    # ==========================================================
    # 1. GROUND LOGIC
    # ==========================================================
    def check_ground_safety(self, img_data, width, height):
        if img_data is None: return True
        try:
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
        except:
            return True 
        
        half_h = int(height * 0.5)
        crop = img[half_h:, :, :]
        
        blue = crop[:, :, 0].astype(np.float32)
        green = crop[:, :, 1].astype(np.float32)
        red  = crop[:, :, 2].astype(np.float32)
        
        avg_blue = np.mean(blue)
        avg_green = np.mean(green)
        avg_red = np.mean(red)
        
        if (avg_green > avg_blue) and (avg_green > avg_red):
            return True 

        brightness = np.mean(crop[:, :, :3], axis=2)
        blue_red_ratio = blue / (red + 1.0)
        
        mask_abyss = brightness < self.ABSOLUTE_DARK_THRESHOLD
        mask_grey_road = (brightness >= self.ABSOLUTE_DARK_THRESHOLD) & \
                         (brightness < 80) & \
                         (blue_red_ratio < self.BLUE_RATIO_THRESHOLD)
        
        road_pixel_count = np.count_nonzero(mask_abyss | mask_grey_road)
        return road_pixel_count <= self.SAFE_PIXEL_COUNT

    # ==========================================================
    # 2. TRAFFIC LIGHT LOGIC (WITH VISUAL DEBUG)
    # ==========================================================
    def scan_for_traffic_lights(self, img_data, width, height):
        if self.model is None or img_data is None: 
            return {'found': False, 'color': 'none', 'center_x': 0.5, 'box_width': 0}

        try:
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
            img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        except:
            return {'found': False, 'color': 'none', 'center_x': 0.5, 'box_width': 0}

        # UPDATED: Lower confidence to catch faint lights (0.3)
        results = self.model(img_bgr, verbose=False, conf=0.3)
        
        best_light = None
        max_area = 0
        debug_img = img_bgr.copy() # Copy for drawing

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                if cls_id == self.TRAFFIC_LIGHT_CLASS_ID:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # FILTERS
                    box_h = y2 - y1
                    box_w = x2 - x1
                    aspect_ratio = box_h / float(box_w + 1e-6)
                    center_y = (y1 + y2) / 2
                    
                    # Draw ALL candidates in GREY first
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (100, 100, 100), 1)

                    # Filter 1: Aspect Ratio (Must be somewhat tall)
                    if aspect_ratio < 1.2: # Relaxed slightly from 1.5
                        continue 
                    
                    # Filter 2: Position (Must be in top 65% of image)
                    if center_y > (height * 0.65):
                        continue

                    # If valid, color it BLUE for now
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    
                    area = box_w * box_h
                    if area > max_area:
                        max_area = area
                        best_light = (x1, y1, x2, y2)

        final_data = {'found': False, 'color': 'none', 'center_x': 0.5, 'box_width': 0}

        if best_light:
            x1, y1, x2, y2 = best_light
            roi = img_bgr[y1:y2, x1:x2]
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            green_mask = cv2.inRange(hsv_roi, self.GREEN_LOWER, self.GREEN_UPPER)
            mask_r1 = cv2.inRange(hsv_roi, self.RED_LOWER1, self.RED_UPPER1)
            mask_r2 = cv2.inRange(hsv_roi, self.RED_LOWER2, self.RED_UPPER2)
            red_mask = mask_r1 | mask_r2
            
            green_count = np.count_nonzero(green_mask)
            red_count = np.count_nonzero(red_mask)
            
            color = 'unknown'
            color_bgr = (0, 255, 255) # Yellow for unknown

            if red_count > green_count and red_count > 5:
                color = 'red'
                color_bgr = (0, 0, 255)
            elif green_count > red_count and green_count > 5:
                color = 'green'
                color_bgr = (0, 255, 0)
            
            # Draw the FINAL chosen box with its color
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), color_bgr, 3)
            cv2.putText(debug_img, color.upper(), (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

            final_data = {
                'found': True,
                'color': color,
                'center_x': ((x1 + x2) / 2) / width,
                'box_width': (x2 - x1) / width
            }

        # --- SHOW DEBUG WINDOW ---
        cv2.imshow("Robot Eye (YOLO)", debug_img)
        cv2.waitKey(1) 
        # -------------------------

        return final_data

    # ==========================================================
    # 3. CROSSWALK LOGIC
    # ==========================================================
    def detect_crosswalk(self, img_data, width, height):
        if img_data is None: return False
        try:
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
            img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            crop = img_bgr[int(height*0.6):, :, :]
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(hsv, self.YELLOW_LOWER, self.YELLOW_UPPER)
            count = np.count_nonzero(mask_yellow)
            total = crop.shape[0] * crop.shape[1]
            return (count / total) > 0.05
        except:
            return False