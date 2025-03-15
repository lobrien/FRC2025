import commands2
from commands2.button import CommandXboxController
from wpilib import SmartDashboard, SendableChooser
from wpimath import applyDeadband
from wpimath.geometry import Pose2d

from commands.algae_idle_command import AlgaeIdle
from commands.algae_intake_command import AlgaeIntake
from commands.algae_outtake_command import AlgaeOuttake
from commands.autonomous_commands import Autos
from commands.coral_idle_command import CoralIdle
from commands.coral_intake_command import CoralIntake
from commands.coral_outtake_command import CoralOuttake
from commands.drive_with_joystick_command import DriveWithJoystickCommand
from commands.print_something_command import PrintSomethingCommand
from commands.reset_gyro_command import ResetGyroCommand
from commands.turn_to_angle_command import TurnToAngleCommand
from constants.autoconsts import AutoConsts
from constants.operatorinterfaceconstants import OperatorInterfaceConstants
from subsystems.algae_subsystem import AlgaeSubsystem
from subsystems.coral_subsystem import CoralSubsystem
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.elevator_subsystem import ElevatorSubsystem
from commands.elevator_command import ElevatorMoveToGoalHeightContinuously
from commands.elevator_init_and_idle_command import ElevatorInitAndIdle
from commands.elevator_down_command import ElevatorDown
from commands.elevator_up_command import ElevatorUp
from constants.elevatorconstants import ElevatorConstants
# from commands.vision_auto_alignment_command import VisionAutoAlign
# from subsystems.vision_subsystem import VisionSubsystem


# The `RobotContainer` class is where the robot's structure and behavior are defined.
# It is the glue that holds the robot together.
# Hardware-wise:
# * The `RobotContainer` has group of Subsystems. When you add or delete a complete
#   subsystem, you will need to update the `RobotContainer`.
# * The `RobotContainer` has one or more input controllers (e.g. Xbox controllers, joysticks).
#
# Command-wise:
# * The `RobotContainer` instantiates the Commands and CommandGroups that make
#   up the robot's behavior. These commands are created and wired together here
#  in the `RobotContainer`. If you ever want to review the robot's behavior, the
#  `RobotContainer` is the proper place to look. Even the autonomous and default commands
#   should be instantiated here.
# * The `RobotContainer` maps between the input devices and the Commands that operate on the Subsystems.
class RobotContainer:
    def __init__(self):
        self.drive_subsystem = DriveSubsystem()
        self.elevator_subsystem = ElevatorSubsystem()
        self.coral_subsystem = CoralSubsystem()
        self.algae_subsystem = AlgaeSubsystem()
        #self.vision_subsystem = VisionSubsystem()

        self.dr_controller = self._initialize_dr_controller()
        self.op_controller = self._initialize_op_controller()

        self._initialize_default_commands()

        self.auto_chooser = self._initialize_shuffleboard()
        # Add chooser to SmartDashboard
        SmartDashboard.putData("Auto Command Selector", self.auto_chooser)

    def _initialize_default_commands(self):
        self.drive_subsystem.setDefaultCommand(DriveWithJoystickCommand(self.drive_subsystem, self.get_drive_value_from_joystick))

        self.algae_subsystem.setDefaultCommand(AlgaeIdle(algae=self.algae_subsystem))

        self.coral_subsystem.setDefaultCommand(CoralIdle(coral=self.coral_subsystem))

        self.elevator_subsystem.setDefaultCommand(
            ElevatorInitAndIdle(elevator=self.elevator_subsystem)
        )


    @staticmethod
    def _initialize_shuffleboard():
        # Auto chooser
        auto_chooser = SendableChooser()
        auto_chooser.setDefaultOption("Forward", AutoConsts.FORWARD)

        # Add options
        auto_chooser.addOption("Side Step", AutoConsts.SIDE_STEP)
        auto_chooser.addOption("Sequence", AutoConsts.SEQUENCE)
        auto_chooser.addOption("Mid Takeout Algae", AutoConsts.MID_TAKEOUT_ALGAE)
        return auto_chooser

    def get_auto_command(self) -> commands2.Command:
        return Autos.forward_and_takeout_algae(self.drive_subsystem)
        # auto_reader = self.auto_chooser.getSelected()
        #
        # if (
        #     auto_reader == AutoConsts.FORWARD
        # ):  # checks which Autonomous command is being used
        #     return Autos.forward(self.drive_subsystem)
        # elif auto_reader == AutoConsts.SIDE_STEP:
        #     return Autos.side_step(self.drive_subsystem)
        # elif auto_reader == AutoConsts.SEQUENCE:  # added new Auto Command
        #     return Autos.goal_sequence(
        #         self.drive_subsystem, [Pose2d(36, 0, 10), Pose2d(0, 48, 0)])
        # elif auto_reader == AutoConsts.MID_TAKEOUT_ALGAE:
        #     return Autos.forward_and_takeout_algae(self.drive_subsystem)
            

    # def get_teleop_command(self):
    #     return DriveWithJoystickCommand(
    #         drive=self.drive_subsystem, drive_percent_fn=self.get_drive_value_from_joystick
    #     )

    def get_drive_value_from_joystick(self) -> tuple[float, float, float]:
        """
        Gets joystick values and scales them for improved operator control.
        Returns:
            Tuple of percentage values in the three joystick axes, leftX, leftY, rightX.
        """
        x_percent = applyDeadband(
            value=self.dr_controller.getLeftX(), deadband=0.1
        )  # Apply deadband to the values above
        y_percent = applyDeadband(value=self.dr_controller.getLeftY(), deadband=0.1)
        rot_percent = applyDeadband(value=self.dr_controller.getRightX(), deadband=0.1)

        x_percent = self.joystick_scaling(x_percent)
        y_percent = self.joystick_scaling(y_percent)
        rot_percent = self.joystick_scaling(rot_percent)

        return (
            x_percent,
            y_percent,
            rot_percent,
        )  # Gives us these variables when we call this function

    @staticmethod
    def joystick_scaling(
        input,
    ):  # this function helps bring an exponential curve in the joystick value and near the zero value it uses less value and is more flat
        a = 1
        output = a * input * input * input + (1 - a) * input
        return output

    def _initialize_dr_controller(self):
        """Initialize the driver controller"""
        controller = CommandXboxController(
            OperatorInterfaceConstants.DRIVER_CONTROLLER_PORT
        )

        # TODO: These should be updated relative to robot's initial field-relative position and angle
        AUTOALIGN_X = 0
        AUTOALIGN_Y = 10
        AUTOALIGN_ANGLE = 45
        # controller.a().onTrue(VisionAutoAlign(
        #     drive=self.drive_subsystem,
        #     desired_field_relative_position_inches=(AUTOALIGN_X, AUTOALIGN_Y),
        #     desired_field_relative_angle_degrees=AUTOALIGN_ANGLE))
        controller.b().whileTrue(
            TurnToAngleCommand(self.drive_subsystem, lambda: True)
        )  # for quick test

        controller.leftBumper().and_(controller.rightBumper()).whileTrue(
            ResetGyroCommand(self.drive_subsystem)
        )

        return controller

    def _initialize_op_controller(self):
        """Initialize the operator controller"""
        controller = CommandXboxController(
            OperatorInterfaceConstants.OPERATOR_CONTROLLER_PORT
        )

        controller.a().onTrue(CoralIntake(coral=self.coral_subsystem))
        controller.b().whileTrue(CoralOuttake(coral=self.coral_subsystem))

        # controller.x().onTrue(AlgaeIntake(algae=self.algae_subsystem))
        # controller.y().onTrue(AlgaeOuttake(algae=self.algae_subsystem))
        # controller.y().onFalse(AlgaeIdle(algae=self.algae_subsystem))

        controller.leftStick().whileTrue(ElevatorUp(self.elevator_subsystem))
        controller.rightStick().whileTrue(ElevatorDown(self.elevator_subsystem))

        controller.leftBumper().onTrue(ElevatorMoveToGoalHeightContinuously(ElevatorConstants.LEVEL_THREE, elev=self.elevator_subsystem))
        controller.leftTrigger().onTrue(ElevatorMoveToGoalHeightContinuously(ElevatorConstants.HOME, elev=self.elevator_subsystem))
        controller.rightTrigger().onTrue(ElevatorMoveToGoalHeightContinuously(ElevatorConstants.FEEDER, elev=self.elevator_subsystem))
        controller.rightBumper().onTrue(ElevatorMoveToGoalHeightContinuously(ElevatorConstants.CLIMB, elev=self.elevator_subsystem))

        return controller

    def periodic(self):
        for subsystem in [
            self.drive_subsystem,
            self.elevator_subsystem,
            self.coral_subsystem,
            self.algae_subsystem,
        ]:
            subsystem.periodic()
