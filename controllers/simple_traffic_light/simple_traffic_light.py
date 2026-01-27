from controller import Robot

class SimpleTrafficLight (Robot):
    def __init__(self):
        super(SimpleTrafficLight, self).__init__()
        self.timestep = int(self.getBasicTimeStep())
        
        # Get devices by their specific names in GenericTrafficLight
        self.red_light = self.getDevice("red light")
        self.green_light = self.getDevice("green light")
        self.orange_light = self.getDevice("orange light") # We get it just to turn it off
        
        # CONFIG
        self.phase_time = 30.0 # Seconds
        self.timer = 0.0
        self.is_green = False

        # INITIAL STATE (Start Red)
        self.red_light.set(1)    # ON
        self.green_light.set(0)  # OFF
        self.orange_light.set(0) # OFF
        self.setCustomData("red") # Helper for external tools

    def run(self):
        while self.step(self.timestep) != -1:
            # Increase timer by timestep (in seconds)
            self.timer += self.timestep / 1000.0
            
            if self.timer >= self.phase_time:
                self.toggle()
                self.timer = 0.0

    def toggle(self):
        self.is_green = not self.is_green
        
        if self.is_green:
            self.red_light.set(0)
            self.green_light.set(1)
            self.setCustomData("green")
        else:
            self.red_light.set(1)
            self.green_light.set(0)
            self.setCustomData("red")

# Run the controller
controller = SimpleTrafficLight()
controller.run()