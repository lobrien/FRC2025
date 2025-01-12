# This class is a container for all the robot's subsystems. This class will change only occasionally, when you
# add new subsystems or commands.
from commands2.button import JoystickButton
from wpilib import XboxController

from commands.autonomous_command import RunAutonomous
from commands.drive.drivewithjoysticks import DriveWithJoysticks

from constants.operatorinterfaceconstants import OperatorInterfaceConstants

from subsystems.drive import DriveSubsystem

import ntcore

# This class is a container for all the robot's subsystems. This class will change as new subsystems are added or removed.
class RobotSystems:
    def __init__(self):
        self.drive = DriveSubsystem()

        self.subsystems = [self.drive]

        # Initialize Controllers (These aren't subsystems because they're single pieces of hardware)
        self.driver_controller = XboxController(OperatorInterfaceConstants.DRIVER_CONTROLLER_PORT)
        self.operator_controller = XboxController(OperatorInterfaceConstants.OPERATOR_CONTROLLER_PORT)

        # Set Default Commands
        self.setDefaultCommands()

        # Configure Button Bindings
        self.configureButtonBindings()

    def setDefaultCommands(self) -> None:
        """
        Set default commands for each subsystem.
        """
        self.drive.setDefaultCommand(DriveWithJoysticks(self.drive, self.driver_controller))

    def configureButtonBindings(self) -> None:
        """
        Map controller buttons to specific commands.
        """
        # Driver Controller Bindings

        # operator Controller Bindings
        #JoystickButton(self.operator_controller, OperatorInterfaceConstants.BUTTON_A).onTrue(StopShooter(self.shooter))

    def getAutonomousCommand(self):
        """
        Returns the autonomous command to be scheduled during autonomous mode.
        """
        return RunAutonomous(self.drive)

    def cancelAutonomousCommand(self) -> None:
        """
        Cancels the autonomous command if it is still running.
        """
        if self.getAutonomousCommand() and self.getAutonomousCommand().isScheduled():
            self.getAutonomousCommand().cancel()

    def setTeleopDefaultCommands(self) -> None:
        """
        Resets default commands during teleop mode (optional customization).
        """
        self.setDefaultCommands()

    def __iter__(self):
        return iter(self.subsystems)
