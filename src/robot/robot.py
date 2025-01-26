import commands2
import commands2.command
import wpilib 
import wpimath
from commands2 import CommandScheduler

import robot_container
from subsystems import drive_subsystem
from constants.operatorinterfaceconstants import OperatorInterfaceConstants

# The `Robot` class is the main robot class. It is responsible for
# the robot lifecycle methods. Because this is a command-based robot,
# a lot of the robot lifecycle is handled by the `CommandScheduler`
# and the majority of lifecycle methods are empty (`pass`).
#
# The `Robot` class *has-a* `RobotContainer`. The `RobotContainer` is
# responsible for customizing the robot.
#
###### CAUTION! CAUTION! ###### CAUTION! CAUTION! ######
# This class is unlikely to require modification.
class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = robot_container.RobotContainer()

    # Robot overall lifecycle methods
    def robotInit(self):
        pass

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    def robotEnd(self):
        pass

    # Autonomous lifecycle methods
    def autonomousInit(self):
        self.autonomous_command = self.container.get_auto_command()
        if self.autonomous_command is not None:
            self.autonomous_command.schedule()

    def autonomousPeriodic(self):
        pass

    def autonomousEnd(self):
        pass

    # Teleop lifecycle methods
    def teleopInit(self):
        if self.autonomous_command is not None:
            self.autonomous_command.cancel()

    def teleopPeriodic(self):
        pass

    def teleopEnd(self):
        pass

    # Test lifecycle methods
    def testInit(self):
        # Cancel all running commands
        CommandScheduler.getInstance().cancelAll();

    def testPeriodic(self):
        pass

    def testEnd(self):
        pass