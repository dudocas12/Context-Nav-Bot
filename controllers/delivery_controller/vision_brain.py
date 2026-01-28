import cv2
import numpy as np

class RobotVision:
    def __init__(self):
        # CONFIGURATION
        # Pixel brightness (0=Black, 255=White). 
        # Road is ~20-30. Sidewalk is ~150. Yellow is ~200.
        self.BLACK_THRESHOLD = 40 
        
        # If we see more than this many black pixels, it's a road.
        self.SAFE_PIXEL_COUNT = 300 

    def check_ground_safety(self, img_data, width, height):
        """
        Input: Raw Webots image bytes.
        Output: True (Safe) or False (Road detected).
        """
        if img_data is None: return True

        # 1. Convert Raw Bytes to Image
        # Webots camera provides 4 bytes per pixel (B, G, R, A)
        try:
            img = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
        except:
            return True # Fallback if image is glitchy

        # 2. Convert to Grayscale (Simpler/Faster)
        gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        
        # 3. Focus on the bottom half (Closest to the wheels)
        # We ignore the top half because it looks too far ahead.
        scan_area = gray[int(height * 0.5) :, :]
        
        # 4. Count "Danger" Pixels
        black_pixels = np.sum(scan_area < self.BLACK_THRESHOLD)
        
        # 5. Decision
        if black_pixels > self.SAFE_PIXEL_COUNT:
            return False # DANGER! Road detected.
        
        return True # Safe (Sidewalk or Yellow Crosswalk)