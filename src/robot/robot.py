import commands2.command
from commands2 import CommandScheduler

import robot_container

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
    def __init__(self):
        super().__init__()

        # The container is the place where all of the robot's components and commands are created.w
        self.container = robot_container.RobotContainer()
        self.autonomous_command = None
        if Robot.isReal():
            print("Robot is real")
        else:
            print("Robot is not real")
        #breakpoint()

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
            self.autonomous_command.schedule() # Note that this is a single command for all of autonomous

    def autonomousPeriodic(self):
        pass

    def autonomousEnd(self):
        pass

    def disabledPeriodic(self):
        self.autonomousCommand = self.container.get_auto_command()

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