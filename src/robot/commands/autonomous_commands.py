import commands2
import commands2.cmd
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import inchesToMeters

from subsystems.drive_subsystem import DriveSubsystem
from commands.drive_to_goal import DriveToGoal
from subsystems.elevator_subsystem import ElevatorSubsystem
from commands.elevator_command import ElevatorMoveToGoalHeightContinuously
from commands.auto_coral_outtake_command import AutoCoralOuttake
from constants.elevatorconstants import ElevatorConstants
from subsystems.coral_subsystem import CoralSubsystem
from commands.print_something_command import PrintSomethingCommand
from commands.coral_idle_command import CoralIdle


class Autos:
    """Class to hold autonomous command factories"""

    def __init__(self):
        raise Exception("This is a utility class, don't make instances of it.")

    @staticmethod
    def side_step(drive: DriveSubsystem):
        """Autonomous routine that drives forward, waits, then moves left."""
        return commands2.cmd.sequence(
            DriveToGoal(drive, Pose2d(inchesToMeters(36), 0, 0)),
            commands2.WaitCommand(1),
            DriveToGoal(drive, Pose2d(0, inchesToMeters(48), 0)),
        )

    @staticmethod
    def goal_sequence(drive: DriveSubsystem, poses: list[Pose2d]):
        """Autonomous routine that drives to a list of poses"""
        return commands2.cmd.sequence(*[DriveToGoal(drive, pose) for pose in poses])

    @staticmethod
    def forward(drive: DriveSubsystem):
        """Autonomous routine that drives forward"""
        return DriveToGoal(drive, Pose2d(inchesToMeters(92), inchesToMeters(0.0), Rotation2d(0.0)))

    def forward_and_takeout_algae(
        drive: DriveSubsystem,
        coral: CoralSubsystem
    ):
        """Autonomous routine that drives forward and shoots coral into level 1, then knocks out algae from reef. 

            We used the command .andThen to properly chain code and used the \ to make the code more readable
        """
        return DriveToGoal(drive_subsystem = drive, goal_pose = Pose2d(inchesToMeters(93.5), inchesToMeters(-16.75), Rotation2d(0.0))) \
            .andThen(AutoCoralOuttake(coral = coral)) \
            .andThen(DriveToGoal(drive_subsystem = drive, goal_pose = Pose2d(inchesToMeters(93.5), inchesToMeters(37.75), Rotation2d(0.0))))
            
