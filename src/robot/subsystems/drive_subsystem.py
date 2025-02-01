import commands2
import phoenix6
import wpilib
import wpimath
from wpilib import SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry
from wpimath.units import degrees, radians, meters, inches, metersToInches

from constants.driveconstants import DriveConstants
from subsystems.swerve_module import SwerveModule


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
            SwerveModule(DriveConstants.DRIVE_FR, DriveConstants.TURN_FR, DriveConstants.CAN_FR, DriveConstants.FR_OFFSET),
            SwerveModule(DriveConstants.DRIVE_FL, DriveConstants.TURN_FL, DriveConstants.CAN_FL, DriveConstants.FL_OFFSET),
            SwerveModule(DriveConstants.DRIVE_BL, DriveConstants.TURN_BL, DriveConstants.CAN_BL, DriveConstants.BL_OFFSET),
            SwerveModule(DriveConstants.DRIVE_BR, DriveConstants.TURN_BR, DriveConstants.CAN_BR, DriveConstants.BR_OFFSET),
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

        self.kinematics = SwerveDrive4Kinematics(*self._get_module_translations())

        self.odometry = self._initialize_odometry(kinematics=self.kinematics)

    def _initialize_odometry(self, kinematics) -> SwerveDrive4Odometry:
        module_positions = [module.get_position() for module in self.modules]
        return SwerveDrive4Odometry(
            kinematics=kinematics,
            gyroAngle=self.get_drive_angle_rotation2d(),
            modulePositions=[module.get_position() for module in self.modules]
        )

    def _get_module_translations(self) -> list[wpimath.geometry.Translation2d]:
        """
        Returns the physical positions of each swerve module relative to the center of the robot.
        The order should match the order of modules in self.modules:
        [FrontRight, FrontLeft, BackLeft, BackRight]

        Returns:
            list[Translation2d]: List of module positions in meters
        """

        # TODO: Is inches correct unit?
        half_length = metersToInches(DriveConstants.WHEELBASE_HALF_LENGTH)# Convert to inches
        half_width = metersToInches(DriveConstants.TRACK_HALF_WIDTH) # Convert to inches

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
            module.set_drive_effort(drive_speed)
            module.set_turn_effort(turn_speed)

    def set_drive_angle(self, desired_angle_degrees):
        # Probably:
        for module in self.modules:
           module.set_turn_angle(desired_angle_degrees)

    def get_drive_angle_degrees(self) -> float:
        # Probably from the gyro? Or kinematics? For now, just return the BL module's angle
        return self.BackLeftModule.get_turn_angle_degrees()

    def get_drive_angle_rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_drive_angle_degrees())

    # This periodic function is called every 20ms during the robotPeriodic phase
    # *in all modes*. It is called automatically by the Commands2 framework.
    def periodic(self):
        wpilib.SmartDashboard.putString('BL Motor pos', 'rotations: {:5.1f}'.format(self.BackLeftModule.get_turn_angle()))
        wpilib.SmartDashboard.putString('BL Can coder pos', 'rotations: {:5.1f}'.format(self.BackLeftModule._get_can_coder_pos()))

        wpilib.SmartDashboard.putString('BR Motor pos', 'rotations: {:5.1f}'.format(self.BackRightModule.get_turn_angle()))
        wpilib.SmartDashboard.putString('BR Can coder pos', 'rotations: {:5.1f}'.format(self.BackRightModule._get_can_coder_pos()))

        wpilib.SmartDashboard.putString('FL Motor pos', 'rotations: {:5.1f}'.format(self.FrontLeftModule.get_turn_angle()))
        wpilib.SmartDashboard.putString('FL Can coder pos', 'rotations: {:5.1f}'.format(self.FrontLeftModule._get_can_coder_pos()))

        wpilib.SmartDashboard.putString('FR Motor pos', 'rotations: {:5.1f}'.format(self.FrontRightModule.get_turn_angle()))
        wpilib.SmartDashboard.putString('FR Can coder pos', 'rotations: {:5.1f}'.format(self.FrontRightModule._get_can_coder_pos()))

        # Update the odometry
        positions = [module.get_position() for module in self.modules]
        self.odometry.update(self.get_drive_angle_rotation2d(), tuple(positions))

        #Update the dashboard
        pose = self.odometry.getPose()
        SmartDashboard.putNumber("Robot X", pose.X())
        SmartDashboard.putNumber("Robot Y", pose.Y())
        SmartDashboard.putNumber("Robot Heading", pose.rotation().degrees())
        for name, module in zip(["FrontLeft", "FrontRight", "BackLeft", "BackRight"],[self.FrontLeftModule, self.FrontRightModule, self.BackLeftModule, self.BackRightModule]):
            state = module.get_state()
            SmartDashboard.putNumber(f"{name} Speed", state.speed)
            SmartDashboard.putNumber(f"{name} Angle", state.angle.degrees())
