import phoenix6
import commands2
import wpimath
import wpilib
import math

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

        # self.pose_estimator = self._initialize_pose_estimator()   #???

    # def _initialize_pose_estimator(self) -> SwerveDrive4PoseEstimator:  #???
    #     kinematics = SwerveDrive4Kinematics(
    #         self._get_module_translations()
    #     )
    #     pose_estimator = SwerveDrive4PoseEstimator(
    #         kinematics=kinematics,
    #         gyro_angle=0,
    #         initial_pose=geometry.Pose2d(),
    #     )
    #     return pose_estimator

    # def _get_module_translations(self) -> list[geometry.Translation2d]:  #???
    #     states = []
    #     for module in self.modules:
    #         states.append(module.get_location())

    # Sets the drive to the given speed and rotation, expressed as percentages
    # of full speed. The speed and rotation values range from -1 to 1.
    # Note that the drive will continue at those values until told otherwise
    def drive(self, drive_speed:float, turn_speed:float) -> None:
        for module in self.modules:
            module.set_drive_speed(drive_speed)
            module.set_turn_speed(turn_speed)

    def set_drive_angle(self, desired_angle_degrees):
        # Hard-coded solely to BL module for now, since that's what we have running
        self.BackLeftModule.set_turn_angle(desired_angle_degrees)
        # Probably:
        #for module in self.modules:
        #    module.set_turn_angle_degrees(desired_angle_degrees)

    def periodic(self):
        wpilib.SmartDashboard.putString('Motor pos', 'rotations: {:5.1f}'.format(self.BackLeftModule.get_turn_angle()))
        wpilib.SmartDashboard.putString('Can coder pos', 'rotations: {:5.1f}'.format(self.BackLeftModule._get_can_coder_pos()))

    
