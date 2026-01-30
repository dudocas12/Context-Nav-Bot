import cv2
import numpy as np
from ultralytics import YOLO

class RobotVision:
    def __init__(self):
        # ==============================================================================
        # ⚙️ CONFIGURATION
        # ==============================================================================
        
        # 1. Road Detection Thresholds
        self.ABSOLUTE_DARK_THRESHOLD = 25 
        self.BLUE_RATIO_THRESHOLD = 1.3
        self.SAFE_PIXEL_COUNT = 200
        
        # 2. Grass Override Threshold
        # Green must be at least 5 units brighter than Blue
        self.GRASS_GREEN_BIAS = 5.0 
        
        print("🧠 LOADING YOLO MODEL (yolo26n.pt)...")
        try:
            self.model = YOLO('yolo26n.pt') 
            print("✅ YOLO Model Loaded")
        except Exception as e:
            print(f"❌ YOLO Failed: {e}")
            self.model = None

        self.TRAFFIC_LIGHT_CLASS_ID = 9
        # 3. Crosswalk Thresholds (Yellow in HSV)
        self.YELLOW_LOWER = np.array([20, 100, 100])
        self.YELLOW_UPPER = np.array([40, 255, 255])

    # ==============================================================================
    # 🛠️ HELPER: PROCESS IMAGE
    # ==============================================================================
    def _process_image(self, img_data, width, height):
        """Converts raw bytes to a BGR numpy array and crops the bottom 50%."""
        if img_data is None: return None
        try:
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
            
            # Crop to bottom 50% (The ground)
            half_h = int(height * 0.5)
            crop = img[half_h:, :, :] # Keep RGBA for now
            return crop
        except Exception as e:
            print(f"⚠️ Vision Error: {e}")
            return None

    # ==============================================================================
    # 🌿 MODULE: GRASS DETECTION
    # ==============================================================================
    def _is_grass(self, blue, green, red):
        """Returns True if the average color is dominantly Green."""
        avg_blue = np.mean(blue)
        avg_green = np.mean(green)
        avg_red = np.mean(red)
        
        # Grass Logic: Green > Blue AND Green > Red
        return (avg_green > avg_blue) and (avg_green > avg_red)

    # ==============================================================================
    # 🛣️ MODULE: ROAD/ABYSS DETECTION
    # ==============================================================================
    def _is_safe_surface(self, crop, blue, red):
        """
        Analyzes brightness and color ratios to find Road or Abyss.
        Returns True if Safe, False if Road/Abyss detected.
        """
        # Calculate Brightness
        brightness = np.mean(crop[:, :, :3], axis=2)
        
        # Calculate Blue/Red Ratio
        blue_red_ratio = blue / (red + 1.0)
        
        # A: Pitch Black Abyss
        mask_abyss = brightness < self.ABSOLUTE_DARK_THRESHOLD
        
        # B: Grey Mid-Tones (Roads are neutral, Sidewalks are Blue-ish)
        mask_grey_road = (brightness >= self.ABSOLUTE_DARK_THRESHOLD) & \
                         (brightness < 80) & \
                         (blue_red_ratio < self.BLUE_RATIO_THRESHOLD)
        
        final_road_mask = mask_abyss | mask_grey_road
        
        road_pixel_count = np.count_nonzero(final_road_mask)
        
        # If too many road/abyss pixels, it is unsafe
        if road_pixel_count > self.SAFE_PIXEL_COUNT:
            return False 
            
        return True 

    # ==============================================================================
    # 🧠 MAIN PUBLIC FUNCTION
    # ==============================================================================
    def check_ground_safety(self, img_data, width, height):
        """
        Orchestrates the safety check.
        1. Process Image
        2. Check for Grass (Override)
        3. Check for Road/Abyss
        """
        crop = self._process_image(img_data, width, height)
        if crop is None: return True # Fail safe

        # Extract Channels (Webots is BGR)
        blue = crop[:, :, 0].astype(np.float32)
        green = crop[:, :, 1].astype(np.float32) 
        red  = crop[:, :, 2].astype(np.float32)

        # 1. Check Grass Override
        if self._is_grass(blue, green, red):
            return True # Safe (Grass detected)

        # 2. Check Road/Abyss
        return self._is_safe_surface(crop, blue, red)
    
    # ==============================================================================
    # 🚦 MODULE: TRAFFIC LIGHT SCANNER (Add this new function)
    # ==============================================================================
    def scan_for_traffic_lights(self, img_data, width, height):
        """
        Detects Traffic Lights using YOLO and draws bounding boxes in a popup window.
        """
        if self.model is None or img_data is None: 
            return 

        try:
            # Convert raw Webots image buffer to BGR
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
            img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        except:
            return 

        # Run YOLO Inference
        # conf=0.3 filters out weak detections
        results = self.model(img_bgr, verbose=False, conf=0.3)
        
        # Create a copy so we can draw boxes without modifying the original data
        debug_img = img_bgr.copy()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                
                # Check if it is a Traffic Light (ID 9)
                if cls_id == self.TRAFFIC_LIGHT_CLASS_ID:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Draw Green Box
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Add Label
                    cv2.putText(debug_img, "TRAFFIC LIGHT", (x1, y1 - 5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Open Visual Debug Window
        cv2.imshow("Robot Eyes (YOLO)", debug_img)
        cv2.waitKey(1)

    # ==============================================================================
    # 🦓 MODULE: CROSSWALK DETECTION
    # ==============================================================================
    def detect_crosswalk(self, img_data, width, height):
        if img_data is None: return False
        try:
            # Process Image
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
            img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            
            # Convert to HSV
            hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
            
            # Create Yellow Mask
            mask_yellow = cv2.inRange(hsv, self.YELLOW_LOWER, self.YELLOW_UPPER)
            
            # Count Yellow Pixels
            yellow_pixels = np.count_nonzero(mask_yellow)
            
            # If we see enough yellow (> 5% of image), we are on a crosswalk
            total_pixels = width * height
            return (yellow_pixels / total_pixels) > 0.05
            
        except Exception as e:
            # print(f"Vision Error: {e}")
            return False