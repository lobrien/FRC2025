from wpimath.geometry import Pose2d, Transform2d, Rotation2d
from wpimath.units import inchesToMeters, degreesToRadians

from constants.new_types import inches, degrees


def robot_relative_to_field(
        current_robot_pose : Pose2d,
        robot_relative_x : inches,
        robot_relative_y: inches,
        robot_relative_rotation : degrees) -> Pose2d:
    """
    Converts a robot-relative transform to a field-relative location
    """
    transform = Transform2d(
        x = inchesToMeters(robot_relative_x),
        y = inchesToMeters(robot_relative_y),
        rotation = Rotation2d.fromDegrees(robot_relative_rotation)
    )
    goal_pose = current_robot_pose.transformBy(transform)
    return goal_pose