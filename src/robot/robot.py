import commands2
import wpilib 

import robot_container
from subsystems import drive_subsystem

# Robot Class
class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = robot_container.RobotContainer()
        self.drive_subsystem = drive_subsystem.DriveSubsystem()

    def teleopPeriodic(self):
        pass
