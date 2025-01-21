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
        if self.container.drive_joystick.getLeftY(0.5):
            self.drive_subsystem.drive_motor.set_control(self.drive_subsystem.brake_request)

        if self.container.drive_joystick.getLeftX(0.5):
            self.drive_subsystem.turn_motor.set_control(self.drive_subsystem.brake_request)
