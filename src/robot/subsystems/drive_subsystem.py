from typing import NewType

import commands2
import wpimath
from wpilib import SmartDashboard, Field2d
from wpimath._controls._controls.controller import ProfiledPIDController
from wpimath._controls._controls.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    ChassisSpeeds,
    SwerveModuleState,
)
from wpimath.units import inchesToMeters, degreesToRadians, degrees
from phoenix6.hardware.pigeon2 import Pigeon2

# This is for type hinting. You can use these types to make your code more readable and maintainable.
# There will be a warning (but not an error!) if you try to assign a value of the wrong type to a variable
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

        # Create a swerve module for each corner.
        self.modules = [
            SwerveModule(
                "FrontRight",
                DriveConstants.DRIVE_FR,
                DriveConstants.TURN_FR,
                DriveConstants.CAN_FR,
                DriveConstants.FR_OFFSET,
            ),
            SwerveModule(
                "FrontLeft",
                DriveConstants.DRIVE_FL,
                DriveConstants.TURN_FL,
                DriveConstants.CAN_FL,
                DriveConstants.FL_OFFSET,
            ),
            SwerveModule(
                "BackLeft",
                DriveConstants.DRIVE_BL,
                DriveConstants.TURN_BL,
                DriveConstants.CAN_BL,
                DriveConstants.BL_OFFSET,
            ),
            SwerveModule(
                "BackRight",
                DriveConstants.DRIVE_BR,
                DriveConstants.TURN_BR,
                DriveConstants.CAN_BR,
                DriveConstants.BR_OFFSET,
            ),
        ]
        self.FrontRightModule = self.modules[0]
        self.FrontLeftModule = self.modules[1]
        self.BackLeftModule = self.modules[2]
        self.BackRightModule = self.modules[3]

        # Gyro to determine robot heading (the direction it is pointed).
        self.gyro = Pigeon2(DriveConstants.PIGEON_ID)
        self.gyro.set_yaw(
            0.0
        )  # Assumes that the robot is facing in the same direction as the driver at the start.

        # Initialize kinematics (equations of motion) and odometry (where are we on the field?)
        self.kinematics = SwerveDrive4Kinematics(*self._get_module_translations())
        self.odometry = self._initialize_odometry(kinematics=self.kinematics)

        self.heartbeat = 0

        # Simulation support
        self.field_sim = Field2d()
        SmartDashboard.putData("Field", self.field_sim)

        # PID Controllers for drive
        self.x_controller, self.y_controller, self.rot_controller = (
            self._initialize_pid_controllers()
        )

        # ---------------------------------------------------------------------
        # Set up odometry, that is figuring out how far we have driven. Example:
        # https://github.com/robotpy/examples/blob/main/MecanumBot/drivetrain.py
        # ---------------------------------------------------------------------

        # The "pose" or position and rotation of the robot.  Usually, we will use
        # odometry to estimate this, but we keep it as a member variable to carry
        # carry between different method calls.  Start at zero, facing +x direction,
        # which for poses, is like Translation2d, +x = forward.
        self.pose = Pose2d(0, 0, Rotation2d.fromDegrees(0))

    # --------------------------------------
    # Public methods for debugging, but not production
    # --------------------------------------

    # Sets the drive to the given speed and rotation, expressed as percentages
    # of full speed. The speed and rotation values range from -1 to 1.
    # Note that the drive will continue at those values until told otherwise
    def drive_by_effort(
        self, drive_speed: inches_per_second, turn_speed: degrees_per_second
    ) -> None:
        for module in self.modules:
            module.set_drive_effort(drive_speed)
            module.set_turn_effort(turn_speed)

    def set_drive_angle(self, desired_angle_degrees: degrees) -> None:
        # Probably:
        for module in self.modules:
            module.set_turn_angle(desired_angle_degrees)

    # --------------------------------------
    # Public methods to get status
    # --------------------------------------

    def get_heading_degrees(self) -> degrees:
        """
        Gets the heading of the robot (direction it is pointing) in degrees.
        CCW is positive.
        """
        # TODO: From the gyro. For now, just return the BL module's angle <-- IMPORTANT! DON'T LEAVE OBSOLETE COMMENTS!
        heading = self.gyro.get_yaw().value

        return heading

    def get_heading_rotation2d(self) -> Rotation2d:
        """
        Gets the heading of the robot (direction it is pointing) as a Rotation2D.
        CCW is positive.
        """
        return Rotation2d.fromDegrees(self.get_heading_degrees())

    def get_pose(self) -> wpimath.geometry.Pose2d:
        return self.odometry.getPose()

    def get_controllers_goals(self) -> tuple[float, float, float]:
        return (
            self.x_controller.getGoal().position,
            self.y_controller.getGoal().position,
            self.rot_controller.getGoal().position,
        )

    # --------------------------------------
    # Public methods for core functionality
    # --------------------------------------

    # This periodic function is called every 20ms during the robotPeriodic phase
    # *in all modes*. It is called automatically by the Commands2 framework.
    def periodic(self):
        for module in self.modules:
            module.periodic()

        # Update the odometry
        positions = [module.get_position() for module in self.modules]
        self.odometry.update(self.get_heading_rotation2d(), tuple(positions))

        # Update the dashboard
        pose = self.odometry.getPose()
        SmartDashboard.putNumber("Robot X", pose.X())
        SmartDashboard.putNumber("Robot Y", pose.Y())
        SmartDashboard.putNumber("Gyro Degree", self.get_heading_degrees())
        SmartDashboard.putNumber("Robot Heading", pose.rotation().degrees())
        for name, module in zip(
            ["FrontLeft", "FrontRight", "BackLeft", "BackRight"],
            [
                self.FrontLeftModule,
                self.FrontRightModule,
                self.BackLeftModule,
                self.BackRightModule,
            ],
        ):
            state = module.get_state()
            SmartDashboard.putNumber(f"{name} Speed", state.speed)
            SmartDashboard.putNumber(f"{name} Angle", state.angle.degrees())

        SmartDashboard.putNumber("Heartbeat", self.heartbeat)
        self.heartbeat += 1

        # Update field sim
        self.field_sim.setRobotPose(pose)
        # TODO: Compare to 2024's self.fieldSim.getObject("Swerve Modules").setPoses(self.module_poses)
        # self.field_sim.setModuleStates([module.get_state() for module in self.modules])

    def drive(
        self,
        x_speed_inches_per_second: inches_per_second,
        y_speed_inches_per_second: inches_per_second,
        rot_speed_degrees_per_second: degrees_per_second,
    ) -> None:
        """
        The main method to use to command the drive system.  Uses field-relative
        directions from the human operator's perspective, assuming the robot
        was initialized while facing the same direction as the driver/operator.
        :param x_speed_inches_per_second:    Speed forward (away from the driver)
        :param x_speed_inches_per_second:    Speed to the left (from the driver's perspective)
        :param rot_speed_degrees_per_second: Desired rotational speed, CCW is positive.
        """
        desatured_module_states = self._speeds_to_states(
            x_speed_inches_per_second,
            y_speed_inches_per_second,
            rot_speed_degrees_per_second,
        )
        for module, state in zip(self.modules, desatured_module_states):
            module.set_desired_state(state)

    # --------------------------------------
    # Private methods to compute module states
    # --------------------------------------

    def _speeds_to_states(
        self,
        x_speed: inches_per_second,
        y_speed: inches_per_second,
        rot_speed: degrees_per_second,
    ) -> list[SwerveModuleState]:
        chassis_speeds = self._get_chassis_speeds(
            x_speed_inches_per_second=x_speed,
            y_speed_inches_per_second=y_speed,
            rot_speed_degrees_per_second=rot_speed,
            field_relative=True,
        )
        swerve_module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        desatured_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states,
            inchesToMeters(DriveConstants.MAX_SPEED_INCHES_PER_SECOND),
        )
        return desatured_module_states

    def _get_chassis_speeds(
        self,
        x_speed_inches_per_second: inches_per_second,
        y_speed_inches_per_second: inches_per_second,
        rot_speed_degrees_per_second: degrees_per_second,
        field_relative: bool = True,
    ) -> ChassisSpeeds:
        # ChassisSpeeds expects meters and radians
        x_speed_meters_per_second = inchesToMeters(x_speed_inches_per_second)
        y_speed_meters_per_second = inchesToMeters(y_speed_inches_per_second)
        rot_speed_radians = degreesToRadians(rot_speed_degrees_per_second)
        # TODO: When self.get_heading_rotation2d() was there every 45 degree rotated it was 90 degree off in driving, but then we just set the value negetive it works <-- IS THIS COMMENT HELPFUL?
        # TODO: Does the previous comment describe a bug?
        cs = ChassisSpeeds.fromRobotRelativeSpeeds(
            x_speed_meters_per_second,
            y_speed_meters_per_second,
            rot_speed_radians,
            -self.get_heading_rotation2d(),
        )
        return cs

    # --------------------------------------
    # Private methods for initialization
    # --------------------------------------

    def _initialize_odometry(
        self, kinematics: SwerveDrive4Kinematics
    ) -> SwerveDrive4Odometry:
        return SwerveDrive4Odometry(
            kinematics=kinematics,
            gyroAngle=self.get_heading_rotation2d(),
            modulePositions=[module.get_position() for module in self.modules],
        )

    def _get_module_translations(self) -> list[wpimath.geometry.Translation2d]:
        """
        Returns the physical positions of each swerve module relative to the center of the robot.
        The order should match the order of modules in self.modules:
        [FrontRight, FrontLeft, BackLeft, BackRight]

        Returns:
            list[Translation2d]: List of module positions in inches
        """

        # Create Translation2d objects for each module position
        # The coordinate system is:
        # - Positive x is forward
        # - Positive y is left
        # - Origin (0,0) is at robot center
        translations = [
            wpimath.geometry.Translation2d(
                DriveConstants.WHEELBASE_HALF_LENGTH, -DriveConstants.TRACK_HALF_WIDTH
            ),  # Front Right
            wpimath.geometry.Translation2d(
                DriveConstants.WHEELBASE_HALF_LENGTH, DriveConstants.TRACK_HALF_WIDTH
            ),  # Front Left
            wpimath.geometry.Translation2d(
                -DriveConstants.WHEELBASE_HALF_LENGTH, DriveConstants.TRACK_HALF_WIDTH
            ),  # Back Left
            wpimath.geometry.Translation2d(
                -DriveConstants.WHEELBASE_HALF_LENGTH, -DriveConstants.TRACK_HALF_WIDTH
            ),  # Back Right
        ]

        return translations

    def set_goal_pose(self, goal_pose: Pose2d) -> None:
        """
        Set the goal for upcoming use of PID controllers.
        Call this before using drive_to_goal().
        :param: goal_pose The desired pose to drive to.
        """
        # Reset the PID loops, because we're starting a new trajectory.
        self.reset_pids()
        # Extract each axis to set the goal for the corresponding controller.
        goal_x = goal_pose.X()
        self.x_controller.setGoal(goal_x)

        goal_y = goal_pose.Y()
        self.y_controller.setGoal(goal_y)
        goal_rot = goal_pose.rotation().degrees()
        goal_rot = wpimath.inputModulus(goal_rot, -180, 180)
        self.rot_controller.setGoal(goal_rot)

    def drive_to_goal(self):
        """
        Drive from present pose toward another pose on the field.
        Uses the goal set by set_goal_pose().  Call that method once first.
        """
        # Calculate the "gas pedal" values for each axis.
        present_x = self.pose.X()
        pid_output_x = self.x_controller.calculate(present_x)
        clamped_x = clamp(
            pid_output_x,
            -DriveConstants.MAX_SPEED_INCHES_PER_SECOND,
            DriveConstants.MAX_SPEED_INCHES_PER_SECOND,
        )  # Drive expects between -1 and 1.

        present_y = self.pose.Y()
        pid_output_y = self.y_controller.calculate(present_y)
        clamped_y = clamp(
            pid_output_y,
            -DriveConstants.MAX_SPEED_INCHES_PER_SECOND,
            DriveConstants.MAX_SPEED_INCHES_PER_SECOND,
        )

        present_rot = self.pose.rotation().degrees()
        present_rot = wpimath.inputModulus(present_rot, -180, 180)
        pid_output_rot = self.rot_controller.calculate(present_rot)
        clamped_rot = clamp(
            pid_output_rot,
            -DriveConstants.MAX_DEGREES_PER_SECOND,
            DriveConstants.MAX_DEGREES_PER_SECOND,
        )

        # Send the values to the drive train.
        self.drive(clamped_x, clamped_y, clamped_rot)

    def is_at_goal(self):
        """
        Used with PID loops to determine if the robot is at the target/goal
        position.
        :returns: True if all three axes (X, Y, rotation) are at the goal.
        """
        all_controllers_at_goal = (
            self.x_controller.atGoal()
            and self.y_controller.atGoal()
            and self.rot_controller.atGoal()
        )
        return all_controllers_at_goal 

    def reset_pids(self):
        """
        Reset the state of PID controllers. Useful when starting a new command.
        """
        # Note that this assumes velocities are zero, and so won't work as well
        # if we schedule a command while moving.  To optimize, we'll need to
        # figure out estimated speeds from odometry and call the version of
        # reset() with two parameters.
        self.x_controller.reset(self.pose.X())
        self.y_controller.reset(self.pose.Y())
        self.rot_controller.reset(self.pose.rotation().degrees())

    def _initialize_pid_controllers(
        self,
    ) -> tuple[ProfiledPIDController, ProfiledPIDController, ProfiledPIDController]:
        """
        Initialize the PID controllers for the drive subsystem.
        """
        # ---------------------------------------------------------------------
        # Create PID controllers for each of the three axes (x=forward, y=left,
        # rotation CCW).  These will help us drive to desired positions.
        # A Profiled PID constrains the velocity and acceleration.
        # Tolerances let us determine when we are at the goal position.
        # ---------------------------------------------------------------------

        # Controller for the x direction (+ forward, away from the driver)

        x_controller = ProfiledPIDController(
            DriveConstants.PIDX_KP,
            0,
            0,
            TrapezoidProfile.Constraints(
                DriveConstants.HORIZ_MAX_V, DriveConstants.HORIZ_MAX_A
            ),
        )
        x_controller.setTolerance(
            DriveConstants.HORIZ_POS_TOL, DriveConstants.HORIZ_VEL_TOL
        )

        # Controller for the y direction (+ left, viewed by the driver)
        y_controller = ProfiledPIDController(
            DriveConstants.PIDY_KP,
            0,
            0,
            TrapezoidProfile.Constraints(
                DriveConstants.HORIZ_MAX_V, DriveConstants.HORIZ_MAX_A
            ),
        )
        y_controller.setTolerance(
            DriveConstants.HORIZ_POS_TOL, DriveConstants.HORIZ_VEL_TOL
        )

        # Controller for the rotation direction (+ counterclockwise, viewed from above)
        # Continuous input enabled so that we can turn left or right, whichever is shorter.
        rot_controller = ProfiledPIDController(
            DriveConstants.PID_ROT_KP,
            0,
            0,
            TrapezoidProfile.Constraints(
                DriveConstants.ROT_MAX_V, DriveConstants.ROT_MAX_A
            ),
        )
        rot_controller.setTolerance(
            DriveConstants.ROT_POS_TOL, DriveConstants.ROT_VEL_TOL
        )
        rot_controller.enableContinuousInput(-180, 180)

        return x_controller, y_controller, rot_controller

    def stop(self):
        for module in self.modules:
            module.stop()


def clamp(val, minval, maxval):
    """Returns a number clamped to minval and maxval."""
    return max(min(val, maxval), minval)
