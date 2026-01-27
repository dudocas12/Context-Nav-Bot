import math
from controller import Robot

class Driver:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # --- MOTORS ---
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # --- SENSORS ---
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        # Initialize Cameras
        self.ground_camera = self.robot.getDevice('ground_camera')
        self.ground_camera.enable(self.timestep)
        
        self.front_camera = self.robot.getDevice('front_camera')
        self.front_camera.enable(self.timestep)
        
        # Helper for YOLO (later)
        self.camera_width = 416
        self.camera_height = 416
        
        # --- LIDAR ---
        try:
            self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()
            print("✅ Lidar Enabled")
        except:
            print("⚠️ WARNING: Lidar not found!")
            self.lidar = None
            
        # --- BUMPERS (NEW) ---
        # We try to enable both left and right bumpers.
        self.bumpers = []
        bumper_names = ['bumper', 'bumper_left', 'bumper_right']
        
        for name in bumper_names:
            try:
                device = self.robot.getDevice(name)
                if device:
                    device.enable(self.timestep)
                    self.bumpers.append(device)
                    print(f"✅ Bumper '{name}' Enabled")
            except:
                pass
        
        if not self.bumpers:
            print("⚠️ WARNING: No bumpers found! Check Scene Tree names.")

    def get_pose(self):
        """ Returns (x, y, heading_degrees) """
        g_vals = self.gps.getValues()
        wx = g_vals[0]
        wy = g_vals[1]
        
        c_vals = self.compass.getValues()
        rad = math.atan2(c_vals[0], c_vals[1])
        deg = math.degrees(rad)
        return wx, wy, deg

    def get_lidar_scan(self):
        if self.lidar is None: return [5.0] * 100
        raw_scan = self.lidar.getRangeImage()
        clean_scan = [min(dist, 5.0) if not math.isinf(dist) else 5.0 for dist in raw_scan]
        return clean_scan

    def check_bumpers(self):
        """
        Returns True if ANY bumper is pressed.
        """
        for b in self.bumpers:
            # getValue returns 1.0 if touched, 0.0 otherwise
            if b.getValue() > 0.5: 
                return True
        return False

    def set_speed(self, linear, angular):
        linear = max(min(linear, 10), -10)
        angular = max(min(angular, 5), -5)
        self.left_motor.setVelocity(linear - angular)
        self.right_motor.setVelocity(linear + angular)

    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        
    def step(self):
        return self.robot.step(self.timestep)