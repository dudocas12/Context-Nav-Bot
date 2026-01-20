import math
from controller import Robot

class DeliveryRobot:
    def __init__(self):
        # 1. Initialize Robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # 2. Motor Setup
        # TIAGo Base standard names
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        
        # Configure motors for infinite rotation
        if self.left_motor and self.right_motor:
            self.left_motor.setPosition(float('inf'))
            self.right_motor.setPosition(float('inf'))
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
        else:
            print("❌ ERROR: Motors not found. Check names in Scene Tree.")
        
        # 3. Sensor Setup
        # LIDAR
        self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        if self.lidar:
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()
            print("✅ Lidar connected.")
        else:
            print("⚠️ Lidar not found (Check name: 'Hokuyo URG-04LX-UG01')")

        # CAMERA
        self.camera = self.robot.getDevice('camera')
        if self.camera:
            self.camera.enable(self.timestep)
            print("✅ Camera connected.")
        
        # GPS
        self.gps = self.robot.getDevice('gps')
        if self.gps:
            self.gps.enable(self.timestep)
            print("✅ GPS connected.")
            
        # COMPASS
        self.compass = self.robot.getDevice('compass')
        if self.compass:
            self.compass.enable(self.timestep)
            print("✅ Compass connected.")

    def set_speed(self, linear, angular):
        """
        linear: m/s (forward speed)
        angular: rad/s (turning speed)
        """
        if not self.left_motor or not self.right_motor:
            return

        wheel_radius = 0.0985 
        axle_length = 0.404
        
        v_left = (linear - angular * axle_length / 2.0) / wheel_radius
        v_right = (linear + angular * axle_length / 2.0) / wheel_radius
        
        # Clamp speed to avoid crazy acceleration
        v_left = max(min(v_left, 10), -10)
        v_right = max(min(v_right, 10), -10)
        
        self.left_motor.setVelocity(v_left)
        self.right_motor.setVelocity(v_right)

    def get_lidar_data(self):
        if self.lidar:
            return self.lidar.getRangeImage()
        return []

    def get_pose(self):
        """
        Returns (x, y, theta_degrees)
        """
        if not self.gps or not self.compass:
            return (0, 0, 0)

        # Get Position
        gps_vals = self.gps.getValues()
        x = gps_vals[0]
        y = gps_vals[1]

        # Get Orientation
        compass_vals = self.compass.getValues()
        rad = math.atan2(compass_vals[0], compass_vals[1])
        bearing = (rad - 1.5708) / math.pi * 180.0
        if bearing < 0.0:
            bearing = bearing + 360.0
            
        return (x, y, bearing)

    def step(self):
        return self.robot.step(self.timestep)