import phoenix6
import commands2
import wpimath
import wpilib
import math

from wpimath.geometry import Rotation2d

from constants.driveconstants import DriveConstants
from subsystems.swerve_module import SwerveModule

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import geometry
from phoenix6.controls import NeutralOut, PositionVoltage

# The `DriveSubsystem` class is a `Subsystem` that contains the robot's drive motors and sensors. It
# is responsible for moving the robot around on the field. Public methods exposed by this class
# should make logical sense for *any* kind of drive, whether it be tank, arcade, swerve, or hovercraft.
# For instance, you wouldn't want to expose a `setLeftSpeed` method for a swerve drive, because that
# doesn't make sense for a swerve drive. Instead, you would want to expose a `drive` method that takes
# a speed and a rotation, because that makes sense for *any* kind of drive.
class DriveSubsystem(commands2.Subsystem):  # Name what type of class this is
    def __init__(self):
        super().__init__()  # Allows the class to call parent class

        self.modules = [
            SwerveModule(DriveConstants.DRIVE_FR, DriveConstants.TURN_FR, DriveConstants.CAN_FR),
            SwerveModule(DriveConstants.DRIVE_FL, DriveConstants.TURN_FL, DriveConstants.CAN_FL),
            SwerveModule(DriveConstants.DRIVE_BL, DriveConstants.TURN_BL, DriveConstants.CAN_BL),
            SwerveModule(DriveConstants.DRIVE_BR, DriveConstants.TURN_BR, DriveConstants.CAN_BR),
        ]
        self.FrontRightModule = self.modules[0]
        self.FrontLeftModule = self.modules[1]
        self.BackLeftModule = self.modules[2]
        self.BackRightModule = self.modules[3]

        # Either brake or coast, depending on motor configuration; we chose brake above.
        self.brake_request = phoenix6.controls.NeutralOut()

        # Position request starts at position 0, but can be modified later.
        self.position_request = phoenix6.controls.PositionVoltage(0).with_slot(0)

        # A motion magic (MM) position request. MM smooths the acceleration.
        self.mm_pos_request = phoenix6.controls.MotionMagicVoltage(0).with_slot(1)

        self.pose_estimator = self._initialize_pose_estimator()

    def _initialize_pose_estimator(self) -> SwerveDrive4PoseEstimator:
        kinematics = SwerveDrive4Kinematics(
            *self._get_module_translations()
        )
        module_positions = [module.get_position() for module in self.modules]
        pose_estimator = SwerveDrive4PoseEstimator(
            kinematics=kinematics,
            gyroAngle=Rotation2d(),
            modulePositions=module_positions,
            initialPose=wpimath.geometry.Pose2d(),
        )
        return pose_estimator

    def _get_module_translations(self) -> list[wpimath.geometry.Translation2d]:
        """
        Returns the physical positions of each swerve module relative to the center of the robot.
        The order should match the order of modules in self.modules:
        [FrontRight, FrontLeft, BackLeft, BackRight]

        Returns:
            list[Translation2d]: List of module positions in meters
        """

        # TODO: Check. Are these the right values? Is inches correct unit?
        half_length = DriveConstants.WHEELBASE_HALF_LENGTH * 1/0.0254  # Convert to inches
        half_width = DriveConstants.TRACK_HALF_WIDTH * 1/0.0254  # Convert to inches

        # Create Translation2d objects for each module position
        # The coordinate system is:
        # - Positive x is forward
        # - Positive y is left
        # - Origin (0,0) is at robot center
        translations = [
            wpimath.geometry.Translation2d(half_length, -half_width),  # Front Right
            wpimath.geometry.Translation2d(half_length, half_width),  # Front Left
            wpimath.geometry.Translation2d(-half_length, half_width),  # Back Left
            wpimath.geometry.Translation2d(-half_length, -half_width),  # Back Right
        ]

        return translations

    # Sets the drive to the given speed and rotation, expressed as percentages
    # of full speed. The speed and rotation values range from -1 to 1.
    # Note that the drive will continue at those values until told otherwise
    def drive(self, drive_speed:float, turn_speed:float) -> None:
        for module in self.modules:
            module.set_drive_speed(drive_speed)
            module.set_turn_speed(turn_speed)

    def _get_can_coder(self) -> float: # the _ in front of a function is indicating that this is only should be used in this class NOT ANYWHERE ELSE
        return self.can_coder.get_absolute_position().value   #Because it's a property and not a method, .value doesn't have () at the end
    
    def _get_wheel_degree(self) -> float:   #angle from -180 to 180 of the wheel
        can_coder_angle_pct = self._get_can_coder()
        angle_degrees = 180 * can_coder_angle_pct

        # What about module[i].get_turn_angle()? Would that be better?

        return angle_degrees

    def set_drive_angle(self, desired_angle_degrees):
        # Hard-coded solely to BL module for now, since that's what we have running
        self.BackLeftModule.set_turn_angle(desired_angle_degrees)
        # Probably:
        #for module in self.modules:
        #    module.set_turn_angle_degrees(desired_angle_degrees)

    # This periodic function is called every 20ms during the robotPeriodic phase
    # *in all modes*. It is called automatically by the Commands2 framework.
    def periodic(self):
        wpilib.SmartDashboard.putString('FR pos', 'rotations: {}'.format(self.FrontRightModule.get_position()))
        wpilib.SmartDashboard.putString('FR pos can coder', 'rotations: {}'.format(self.FrontRightModule.can_coder.get_absolute_position()))
