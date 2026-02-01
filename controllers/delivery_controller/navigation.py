import math
from controller import Robot

class Driver:
    '''
    Hardware abstraction layer for the TIAGo robot in Webots.
    Provides simplified access to motors, sensors, cameras, and navigation.
    '''
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # MOTORS
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # SENSORS
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        # Initialize Cameras
        self.ground_camera = self.robot.getDevice('ground_camera')
        self.ground_camera.enable(self.timestep)
        
        self.front_camera = self.robot.getDevice('front_camera')
        self.front_camera.enable(self.timestep)
        
        # Helper for YOLO (unused in this version but kept for compatibility)
        self.camera_width = 416
        self.camera_height = 416
        
        # LIDAR
        try:
            self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()
            print("[INIT] Lidar sensor enabled")
        except:
            print("[WARN] Lidar sensor not found")
            self.lidar = None
            
        # BUMPERS
        self.bumpers = []
        # TIAGo Base only has one device named "bumper"
        bumper_names = ['bumper'] 
        
        for name in bumper_names:
            device = self.robot.getDevice(name)
            if device:
                device.enable(self.timestep)
                self.bumpers.append(device)
                print(f"[INIT] Bumper sensor '{name}' enabled")
        
        if not self.bumpers:
            print("[WARN] No bumper sensors detected")

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
        '''
        Returns a cleaned list of lidar distance readings.
        Infinite values are capped at 5.0 meters for safe processing.
        '''
        if self.lidar is None: return [5.0] * 100
        raw_scan = self.lidar.getRangeImage()
        clean_scan = [min(dist, 5.0) if not math.isinf(dist) else 5.0 for dist in raw_scan]
        return clean_scan

    def check_bumpers(self):
        """ Returns True if ANY bumper is pressed. """
        for b in self.bumpers:
            if b.getValue() > 0.5: 
                return True
        return False

    def set_speed(self, linear, angular):
        '''
        Sets the robot's movement speed using differential drive.
        Linear: forward/backward speed (-10 to 10).
        Angular: rotation speed (-5 to 5), positive = left turn.
        '''
        linear = max(min(linear, 10), -10)
        angular = max(min(angular, 5), -5)
        self.left_motor.setVelocity(linear - angular)
        self.right_motor.setVelocity(linear + angular)

    def stop(self):
        '''Immediately stops all wheel motors.'''
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        
    def step(self):
        '''Advances the simulation by one timestep. Returns -1 if simulation ended.'''
        return self.robot.step(self.timestep)
        
    def get_ground_image(self):
        """Returns the raw image data from the ground camera"""
        return self.ground_camera.getImage()

    def get_front_image(self):
        """Returns the raw image data from the front camera"""
        return self.front_camera.getImage()