import cv2
import numpy as np
from ultralytics import YOLO

class RobotVision:
    def __init__(self):
        # CONFIGURATION
        
        # 1. Road Detection
        self.ABSOLUTE_DARK_THRESHOLD = 25 
        self.BLUE_RATIO_THRESHOLD = 1.3
        self.SAFE_PIXEL_COUNT = 200
        
        # 2. Grass Override
        self.GRASS_GREEN_BIAS = 5.0 
        
        # 3. Crosswalk (Yellow)
        self.YELLOW_LOWER = np.array([20, 100, 100])
        self.YELLOW_UPPER = np.array([40, 255, 255])

        # 4. Traffic Lights (YOLO + Colors) - FIXED MISSING ATTRIBUTES
        print("[INIT] Loading YOLO model (yolo26n.pt)...")
        try:
            self.model = YOLO('yolo26n.pt') 
            print("[INIT] YOLO model loaded successfully")
        except Exception as e:
            print(f"[ERROR] Failed to load YOLO model: {e}")
            self.model = None

        self.TRAFFIC_LIGHT_CLASS_ID = 9
        
        # Color thresholds for traffic light detection
        self.GREEN_LOWER = np.array([35, 40, 40])
        self.GREEN_UPPER = np.array([95, 255, 255])
        
        self.RED_LOWER1 = np.array([0, 50, 50])
        self.RED_UPPER1 = np.array([10, 255, 255])
        self.RED_LOWER2 = np.array([160, 50, 50])
        self.RED_UPPER2 = np.array([180, 255, 255])

    def _process_image(self, img_data, width, height):
        '''
        Converts raw camera byte data into a numpy array and crops to bottom 50%.
        Used for ground-level analysis where only the immediate area matters.
        Returns the cropped image or None if processing fails.
        '''
        if img_data is None: return None
        try:
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
            # Crop to bottom 50% for ground checks
            half_h = int(height * 0.5)
            crop = img[half_h:, :, :] 
            return crop
        except:
            return None

    def check_ground_safety(self, img_data, width, height):
        '''
        Analyzes the ground camera image to detect unsafe terrain (roads, dark surfaces).
        Returns True if the ground is safe to traverse (grass or non-road surface).
        Returns False if a road or dangerous surface is detected ahead.
        '''
        crop = self._process_image(img_data, width, height)
        if crop is None: return True

        blue = crop[:, :, 0].astype(np.float32)
        green = crop[:, :, 1].astype(np.float32) 
        red  = crop[:, :, 2].astype(np.float32)

        # Grass Check
        if (np.mean(green) > np.mean(blue)) and (np.mean(green) > np.mean(red)):
            return True 

        # Road Check
        brightness = np.mean(crop[:, :, :3], axis=2)
        blue_red_ratio = blue / (red + 1.0)
        
        mask_abyss = brightness < self.ABSOLUTE_DARK_THRESHOLD
        mask_grey_road = (brightness >= self.ABSOLUTE_DARK_THRESHOLD) & \
                         (brightness < 80) & \
                         (blue_red_ratio < self.BLUE_RATIO_THRESHOLD)
        
        final_road_mask = mask_abyss | mask_grey_road
        return np.count_nonzero(final_road_mask) <= self.SAFE_PIXEL_COUNT

    def detect_crosswalk(self, img_data, width, height):
        '''
        Detects yellow crosswalk markings in the ground camera image.
        Analyzes the bottom 40% of the image for yellow pixels using HSV thresholds.
        Returns True if more than 5% of pixels match crosswalk color.
        '''
        if img_data is None: return False
        try:
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
            img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
            # Check bottom 40% for yellow
            crop_hsv = hsv[int(height*0.6):, :, :]
            mask = cv2.inRange(crop_hsv, self.YELLOW_LOWER, self.YELLOW_UPPER)
            return (np.count_nonzero(mask) / (crop_hsv.shape[0]*crop_hsv.shape[1])) > 0.05
        except:
            return False

    def scan_for_traffic_lights(self, img_data, width, height):
        '''
        Uses YOLO object detection to find traffic lights in the front camera image.
        Applies geometry filters to reject false positives (e.g., fire hydrants).
        Analyzes the detected light's color using HSV thresholds.
        Returns a dict with: found (bool), color (red/green/unknown), center_x, box_width.
        '''
        # Default return
        result = {'found': False, 'color': 'none', 'center_x': 0.5, 'box_width': 0}
        
        if self.model is None or img_data is None: return result

        try:
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
            img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        except:
            return result

        results = self.model(img_bgr, verbose=False, conf=0.3)
        best_light = None
        max_area = 0
        debug_img = img_bgr.copy()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                if int(box.cls[0]) == self.TRAFFIC_LIGHT_CLASS_ID:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Geometry Filter (Anti-Hydrant)
                    box_h = y2 - y1
                    box_w = x2 - x1
                    if (box_h / float(box_w+1e-6)) < 1.2: continue 
                    if ((y1+y2)/2) > (height * 0.65): continue

                    # Valid Candidate
                    area = box_w * box_h
                    if area > max_area:
                        max_area = area
                        best_light = (x1, y1, x2, y2)
                    
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (100, 100, 100), 1)

        if best_light:
            x1, y1, x2, y2 = best_light
            
            # Color Logic
            roi = img_bgr[y1:y2, x1:x2]
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            green_px = np.count_nonzero(cv2.inRange(hsv_roi, self.GREEN_LOWER, self.GREEN_UPPER))
            red_px = np.count_nonzero(cv2.inRange(hsv_roi, self.RED_LOWER1, self.RED_UPPER1) | 
                                      cv2.inRange(hsv_roi, self.RED_LOWER2, self.RED_UPPER2))
            
            color = 'unknown'
            box_c = (0, 255, 255)
            if red_px > green_px and red_px > 5:
                color = 'red'; box_c = (0, 0, 255)
            elif green_px > red_px and green_px > 5:
                color = 'green'; box_c = (0, 255, 0)

            cv2.rectangle(debug_img, (x1, y1), (x2, y2), box_c, 3)
            result = {'found': True, 'color': color, 'center_x': ((x1+x2)/2)/width, 'box_width': (x2-x1)/width}

        cv2.imshow("Robot Eyes (YOLO)", debug_img)
        cv2.waitKey(1)
        return result