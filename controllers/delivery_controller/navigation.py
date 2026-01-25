import math
from controller import Robot

class Driver:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Motors
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Sensors
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
    def get_pose(self):
        """
        Returns (x, y, heading_degrees).
        Using your diagnostic data: 
        - GPS X/Y are aligned with World X/Y.
        - Compass X is Sin component, Compass Y is Cos component.
        """
        # 1. GPS (Already Correct)
        g_vals = self.gps.getValues()
        wx = g_vals[0]
        wy = g_vals[1]
        
        # 2. COMPASS (Fixed for your sensor rotation)
        c_vals = self.compass.getValues()
        # atan2(Sin, Cos) -> atan2(CompassX, CompassY)
        rad = math.atan2(c_vals[0], c_vals[1])
        deg = math.degrees(rad)
        
        return wx, wy, deg

    def set_speed(self, linear, angular):
        """
        linear: forward speed
        angular: turning speed (positive = left)
        """
        linear = max(min(linear, 10), -10)
        angular = max(min(angular, 5), -5)
        
        # If the robot spins the wrong way, swap the signs of 'angular' below:
        self.left_motor.setVelocity(linear - angular)
        self.right_motor.setVelocity(linear + angular)

    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        
    def step(self):
        return self.robot.step(self.timestep)