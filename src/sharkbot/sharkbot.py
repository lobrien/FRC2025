import logging
import sys
print (sys.path)

import wpilib
import wpilib.drive
import commands2

import ntcore

from robotsystems import RobotSystems
from commands.autonomous_command import RunAutonomous

# This is the "entry point" for the robot code. It's the first thing that runs.
# This file should rarely, if ever, change
class Sharkbot(wpilib.TimedRobot):
    def robotInit(self):
        self.systems = RobotSystems()
        self.autonomous_command = RunAutonomous(self.systems.drive)

    # Every 20ms in all modes
    def robotPeriodic(self):
        logging.debug("robotPeriodic")

        # TODO: flush NetworkTables

        # Log all subsystems
        for system in self.systems:
            system.logPeriodic()

        # Run whatever command is next in the queue
        commands2.CommandScheduler.getInstance().run()

    def autonomousInit(self):
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def teleopInit(self):
        if self.autonomous_command:
            self.autonomous_command.cancel()

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

