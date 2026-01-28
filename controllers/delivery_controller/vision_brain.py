import cv2
import numpy as np

class RobotVision:
    def __init__(self):
        # CONFIGURATION
        
        # 1. Road Detection Thresholds
        self.ABSOLUTE_DARK_THRESHOLD = 25 
        self.BLUE_RATIO_THRESHOLD = 1.3
        self.SAFE_PIXEL_COUNT = 200
        
        # 2. Grass Override Threshold
        # Green must be at least 5 units brighter than Blue
        self.GRASS_GREEN_BIAS = 5.0 

    def check_ground_safety(self, img_data, width, height):
        if img_data is None: return True

        try:
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
        except:
            return True 

        # 1. Split Channels & Crop
        half_h = int(height * 0.5)
        crop = img[half_h:, :, :]
        
        # Webots is BGR
        blue = crop[:, :, 0].astype(np.float32)
        green = crop[:, :, 1].astype(np.float32) 
        red  = crop[:, :, 2].astype(np.float32)
        
        # --- 2. GRASS OVERRIDE CHECK ---
        avg_blue = np.mean(blue)
        avg_green = np.mean(green)
        avg_red = np.mean(red)
        
        # Grass is Green (Green > Blue)
        is_grass = (avg_green > avg_blue) and (avg_green > avg_red)
        
        if is_grass:
            return True # SAFE! (Even if there are dark shadows)

        # --- 3. STANDARD ROAD DETECTION (If not grass) ---
        
        brightness = np.mean(crop[:, :, :3], axis=2)
        blue_red_ratio = blue / (red + 1.0)
        
        # A: Pitch Black Abyss
        mask_abyss = brightness < self.ABSOLUTE_DARK_THRESHOLD
        
        # B: Grey Mid-Tones (Roads are neutral, Sidewalks are Blue-ish)
        mask_grey_road = (brightness >= self.ABSOLUTE_DARK_THRESHOLD) & \
                         (brightness < 80) & \
                         (blue_red_ratio < self.BLUE_RATIO_THRESHOLD)
        
        final_road_mask = mask_abyss | mask_grey_road
        
        road_pixel_count = np.count_nonzero(final_road_mask)
        
        if road_pixel_count > self.SAFE_PIXEL_COUNT:
            return False # Unsafe
        
        return True # Safe