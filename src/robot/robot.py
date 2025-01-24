import commands2
import commands2.command
import wpilib 
import wpimath 

import robot_container
from subsystems import drive_subsystem

# Robot Class
class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = robot_container.RobotContainer()
        self.drive_subsystem = drive_subsystem.DriveSubsystem()

    def disabledPeriodic(self):
        pass

    def autonomousPeriodic(self):
        self.auto_command = self.container.get_auto_commands()

    def teleopPeriodic(self):
        self.teleop_command = self.container.get_teleop_commands()
        self.teleop_command.schedule

        a = self.container.drive_controller.getAButton()

        if a:
            self.drive_subsystem.set_pos_with_degree()
