import commands2
import commands2.command
import wpilib 
import wpimath
from commands2 import CommandScheduler

import robot_container
from subsystems import drive_subsystem
from tests import drive_test
from constants.operatorinterfaceconstants import OperatorInterfaceConstants

# Robot Class
class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = robot_container.RobotContainer()

    ###### CAUTION! CAUTION! ###### CAUTION! CAUTION! ######
    # Do not change anything beneath this line during debugging.
    # It is important to keep the robot periodic and autonomous init methods as they are.

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    def autonomousInit(self):
        self.autonomous_command = self.container.get_auto_command()
        if self.autonomous_command is not None:
            self.autonomous_command.schedule()