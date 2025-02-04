from typing import NewType

import commands2
import wpimath
from wpilib import SmartDashboard, Field2d
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, ChassisSpeeds
from wpimath.units import metersToInches, inchesToMeters, degreesToRadians

# This is for type hinting. You can use these types to make your code more readable and maintainable.
# There will be a warning (but not an error!) if you try to assign a value of the wrong type to a variable
inches = NewType("inches", float)
degrees = NewType("degrees", float)
inches_per_second = NewType("inches_per_second", float)
degrees_per_second = NewType("degrees_per_second", float)


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
            SwerveModule("FrontRight",DriveConstants.DRIVE_FR, DriveConstants.TURN_FR, DriveConstants.CAN_FR, DriveConstants.FR_OFFSET),
            SwerveModule("FrontLeft",DriveConstants.DRIVE_FL, DriveConstants.TURN_FL, DriveConstants.CAN_FL, DriveConstants.FL_OFFSET),
            SwerveModule("BackLeft", DriveConstants.DRIVE_BL, DriveConstants.TURN_BL, DriveConstants.CAN_BL, DriveConstants.BL_OFFSET),
            SwerveModule("BackRight", DriveConstants.DRIVE_BR, DriveConstants.TURN_BR, DriveConstants.CAN_BR, DriveConstants.BR_OFFSET),
        ]
        self.FrontRightModule = self.modules[0]
        self.FrontLeftModule = self.modules[1]
        self.BackLeftModule = self.modules[2]
        self.BackRightModule = self.modules[3]

        self.kinematics = SwerveDrive4Kinematics(*self._get_module_translations())
        self.odometry = self._initialize_odometry(kinematics=self.kinematics)

        self.heartbeat = 0

        # Simulation support
        self.field_sim = Field2d()
        SmartDashboard.putData('Field', self.field_sim)

    # Sets the drive to the given speed and rotation, expressed as percentages
    # of full speed. The speed and rotation values range from -1 to 1.
    # Note that the drive will continue at those values until told otherwise
    def drive_by_effort(self, drive_speed:inches_per_second, turn_speed:degrees_per_second) -> None:
        for module in self.modules:
            module.set_drive_effort(drive_speed)
            module.set_turn_effort(turn_speed)

    def set_drive_angle(self, desired_angle_degrees : degrees) -> None:
        # Probably:
        for module in self.modules:
           module.set_turn_angle(desired_angle_degrees)

    def get_drive_angle_degrees(self) -> degrees:
        # Probably from the gyro? Or kinematics? For now, just return the BL module's angle
        return self.BackLeftModule.get_turn_angle_degrees()

    def get_drive_angle_rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_drive_angle_degrees())

    # This periodic function is called every 20ms during the robotPeriodic phase
    # *in all modes*. It is called automatically by the Commands2 framework.
    def periodic(self):
        for module in self.modules:
            module.periodic()

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

        SmartDashboard.putNumber("Heartbeat", self.heartbeat)
        self.heartbeat += 1

        # Update field sim
        self.field_sim.setRobotPose(pose)
        # TODO: Compare to 2024's self.fieldSim.getObject("Swerve Modules").setPoses(self.module_poses)
        self.field_sim.setModuleStates([module.get_state() for module in self.modules])


    def drive(self, x_speed : inches_per_second, y_speed : inches_per_second, rot_speed : degrees_per_second) -> None:
        desatured_module_states = self._speeds_to_states(x_speed, y_speed, rot_speed)
        for module, state in zip(self.modules, desatured_module_states):
            module.set_desired_state(state, True)

    def get_pose(self) -> wpimath.geometry.Pose2d:
        return self.odometry.getPose()

    def _get_chassis_speeds(self, x_speed_inches_per_second: inches_per_second, y_speed_inches_per_second:  inches_per_second, rot_speed_degrees_per_second: degrees_per_second, field_relative: bool = True) -> ChassisSpeeds:
        # ChassisSpeeds expects meters and radians
        x_speed_meters_per_second = inchesToMeters(x_speed_inches_per_second)
        y_speed_meters_per_second = inchesToMeters(y_speed_inches_per_second)
        rot_speed_radians = degreesToRadians(rot_speed_degrees_per_second)
        if field_relative:
            cs = ChassisSpeeds.fromRobotRelativeSpeeds(x_speed_meters_per_second, y_speed_meters_per_second, rot_speed_radians, self.get_drive_angle_rotation2d())
        else:
            cs = ChassisSpeeds(x_speed_meters_per_second, y_speed_meters_per_second, rot_speed_radians)
        return cs

    def _initialize_odometry(self, kinematics: SwerveDrive4Kinematics) -> SwerveDrive4Odometry:
        return SwerveDrive4Odometry(
            kinematics=kinematics,
            gyroAngle=self.get_drive_angle_rotation2d(),
            modulePositions=[module.get_position() for module in self.modules]
        )

    def _update_odometry(self):
        self.odometry.update(
            self.get_drive_angle_rotation2d(),
            tuple([module.get_position() for module in self.modules])
        )

    def _get_module_translations(self) -> list[wpimath.geometry.Translation2d]:
        """
        Returns the physical positions of each swerve module relative to the center of the robot.
        The order should match the order of modules in self.modules:
        [FrontRight, FrontLeft, BackLeft, BackRight]

        Returns:
            list[Translation2d]: List of module positions in inches
        """

        # Using inches for the module positions
        half_length = metersToInches(DriveConstants.WHEELBASE_HALF_LENGTH)  # Convert to inches
        half_width = metersToInches(DriveConstants.TRACK_HALF_WIDTH)  # Convert to inches

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

    def _speeds_to_states(self, x_speed : inches_per_second, y_speed : inches_per_second, rot_speed : degrees_per_second) -> list[SwerveModule.State]:
        chassis_speeds = self._get_chassis_speeds(x_speed_inches_per_second=x_speed, y_speed_inches_per_second=y_speed, rot_speed=rot_speed, field_relative=True)
        swerve_module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        desatured_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, DriveConstants.MAX_SPEED_INCHES_PER_SECOND)
        return desatured_module_states

