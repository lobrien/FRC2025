import commands2

import robot_container

# Set Deadband and Print Class
class SetDeadbandAndPrint(commands2.Command): 
    def __init__(self):
        self.container = robot_container.RobotContainer()

    def initialize(self):  # Setting function
        self.container.set_controller_deadbands()
    
    def execute(self):  # What actions it does
        self.container.print_driver_joystick_values()

    def isFinished(self):
        pass

    def end(self):  
        pass
