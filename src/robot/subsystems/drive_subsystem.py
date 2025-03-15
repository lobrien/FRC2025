from typing import Optional
import math

from typing import Optional

import commands2
import numpy as np
import wpilib
import wpimath
from wpilib import SmartDashboard, Field2d, RobotBase
from wpimath.estimator import SwerveDrive4PoseEstimator
import logging
from wpilib import SmartDashboard, Field2d
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    # SwerveDrive4Odometry,
    ChassisSpeeds,
    SwerveModuleState,
    SwerveModulePosition,
)
from wpimath.estimator import SwerveDrive4PoseEstimator

from wpimath.units import inchesToMeters, degreesToRadians, degrees, metersToInches
from phoenix6.hardware.pigeon2 import Pigeon2

from constants.driveconstants import DriveConstants
from constants.fieldconstants import FieldConstants
from constants.new_types import inches_per_second, degrees_per_second, percentage
from subsystems.swerve_module import SwerveModule

logger = logging.getLogger(__name__)

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
                (DriveConstants.WHEELBASE_HALF_LENGTH, -DriveConstants.TRACK_HALF_WIDTH),
            ),
            SwerveModule(
                "FrontLeft",
                DriveConstants.DRIVE_FL,
                DriveConstants.TURN_FL,
                DriveConstants.CAN_FL,
                DriveConstants.FL_OFFSET,
                (DriveConstants.WHEELBASE_HALF_LENGTH, DriveConstants.TRACK_HALF_WIDTH),
            ),
            SwerveModule(
                "BackLeft",
                DriveConstants.DRIVE_BL,
                DriveConstants.TURN_BL,
                DriveConstants.CAN_BL,
                DriveConstants.BL_OFFSET,
                (-DriveConstants.WHEELBASE_HALF_LENGTH, DriveConstants.TRACK_HALF_WIDTH),
            ),
            SwerveModule(
                "BackRight",
                DriveConstants.DRIVE_BR,
                DriveConstants.TURN_BR,
                DriveConstants.CAN_BR,
                DriveConstants.BR_OFFSET,
                (-DriveConstants.WHEELBASE_HALF_LENGTH, -DriveConstants.TRACK_HALF_WIDTH),
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

        # Simulation support (must precede kinematics and odometry initialization)
        self.field_sim = Field2d()
        SmartDashboard.putData("Field_Sim", self.field_sim)
        # Initialize simulation variables
        self.sim_pose = Pose2d(
            inchesToMeters(FieldConstants.AUTO_START_X),
            inchesToMeters(FieldConstants.AUTO_START_Y),
            Rotation2d.fromDegrees(0))
        self.prev_sim_time = 0.0


        # Initialize kinematics (equations of motion) and odometry (where are we on the field?)
        self.kinematics = SwerveDrive4Kinematics(*self._get_module_translations())
        self.odometry = self._initialize_odometry(
            kinematics=self.kinematics,
            initial_pose=Pose2d(
                inchesToMeters(FieldConstants.AUTO_START_X),
                inchesToMeters(FieldConstants.AUTO_START_Y),
                Rotation2d.fromDegrees(0)
            )
        )

        self.heartbeat = 0


        # Create module visualization objects
        if wpilib.RobotBase.isSimulation():
            self.module_visualization = self.field_sim.getObject("Swerve Modules")
            self.module_poses = []
            for _ in range(4):
                self.module_poses.append(Pose2d(0, 0, Rotation2d(0)))

        # PID Controllers for drive
        self.x_controller, self.y_controller, self.rot_controller = (
            self._initialize_pid_controllers()
        )

        # ---------------------------------------------------------------------
        # Set up odometry, that is figuring out how far we have driven. Example:
        # https://github.com/robotpy/examples/blob/main/MecanumBot/drivetrain.py
        # ---------------------------------------------------------------------

        # The initial "pose" or position and rotation of the robot.  Usually, we will use
        # odometry to estimate this, but we keep it as a member variable to
        # carry between different method calls.  Start at zero, facing +x direction,
        # which for poses, is like Translation2d, +x = forward.
        self.pose = self.odometry.getEstimatedPosition()

        # Limelight should be part of this subsystem due to interaction with odometry
        # discovered_limelights = limelight.discover_limelights(debug=True)
        # if len(discovered_limelights) == 0:
        #     logger.warning("No Limelight found!")
        #     self.limelight = None
        # else:
        #     logger.info("Found Limelight!")
        #     self.limelight = limelight.Limelight(discovered_limelights[0])
        #     limelight_address = self.limelight.base_url
        #     logger.debug("Limelight address: %s", limelight_address)
        #     status = self.limelight.get_status()
        #     logger.info("Limelight status: %s", status)
        #     self.limelight.enable_websocket()

    # --------------------------------------
    # Public methods for debugging, but not production
    # --------------------------------------

    def drive_by_effort(
            self, drive_effort: percentage, turn_effort: percentage
    ) -> None:
        for module in self.modules:
            module.set_drive_effort(drive_effort)
            module.set_turn_effort(turn_effort)

    def set_drive_angle(self, desired_angle_degrees: degrees) -> None:
        for module in self.modules:
            module.set_turn_angle(desired_angle_degrees)

    # --------------------------------------
    # Public methods to get status
    # --------------------------------------

    def get_gyro_heading_degrees(self) -> degrees:
        """
        Gets the heading of the robot (direction it is pointing) in degrees.
        CCW is positive.
        """
        if wpilib.RobotBase.isSimulation():
            return self.sim_pose.rotation().degrees()
        else:
            heading = self.gyro.get_yaw().value
            return heading

    def get_gyro_heading_rotation2d(self) -> Rotation2d:
        """
        Gets the heading of the robot (direction it is pointing) as a Rotation2D.
        CCW is positive.
        """
        return Rotation2d.fromDegrees(self.get_gyro_heading_degrees())

    def get_pose(self) -> wpimath.geometry.Pose2d:
        """
        Get the current robot pose, either from simulation or odometry.
        """
        if wpilib.RobotBase.isSimulation():
            return self.sim_pose
        else:
            return self.odometry.getEstimatedPosition()

    def get_controllers_goals(self) -> tuple[float, float, float]:
        return (
            self.x_controller.getGoal().position,
            self.y_controller.getGoal().position,
            self.rot_controller.getGoal().position,
        )

    # --------------------------------------
    # Public methods for core functionality
    # --------------------------------------

    def periodic(self):
        """
        Called periodically during all robot modes.
        Updates odometry and dashboard displays.
        """
        for module in self.modules:
            module.periodic()
        if RobotBase.isSimulation():
            self.simulationPeriodic()

        # Update the odometry
        positions = [module.get_position() for module in self.modules]
        self.odometry.update(self.get_gyro_heading_rotation2d(), tuple(positions))

        # # Update with vision
        # maybe_result = self._limelight_periodic()
        # if maybe_result is not None:
        #     self._on_new_vision_result(maybe_result)

        # TODO: This block needs to be cleaned up. We have 3 different poses! Which one is correct?
        current_pos, estimated_pos = self._odometry_periodic()
        # Update pose for user code
        self.pose = estimated_pos  # TODO: Override to estimated seems mistake? Maybe current_pos if RobotBase.isReal() else estimated_pos (?)
        # Update field sim
        self.field_sim.setRobotPose(self.pose)

        self._dashboard_periodic(current_pos, estimated_pos)
        self.heartbeat += 1

    def _dashboard_periodic(self, current_pos, estimated_pos):
        # Update the dashboard
        if RobotBase.isSimulation():
            self.simulationPeriodic()
        else:
            SmartDashboard.putBoolean("Simulation", False)
        self.pose = self.odometry.getEstimatedPosition()
        SmartDashboard.putNumber("Robot X", metersToInches(self.pose.X()))
        SmartDashboard.putNumber("Robot Y", metersToInches(self.pose.Y()))
        SmartDashboard.putNumber("Gyro Degree", self.get_gyro_heading_degrees())
        SmartDashboard.putNumber("Robot Y", metersToInches(self.pose.Y()))
        # Update dashboard data
        SmartDashboard.putBoolean(
            "Odometry Move", not estimated_pos.__eq__(current_pos)
        )
        SmartDashboard.putNumber("Odometry X", estimated_pos.X())
        SmartDashboard.putNumber("Odometry Y", estimated_pos.Y())
        SmartDashboard.putNumber("Odometry Heading", estimated_pos.rotation().degrees())
        # Update additional dashboard data
        SmartDashboard.putNumber("Gyro Degree", self.get_gyro_heading_degrees())
        SmartDashboard.putNumber("Robot Heading", self.pose.rotation().degrees())
        logger.debug(f"Robot X: {metersToInches(self.pose.X())}")
        logger.debug(f"Robot Y: {metersToInches(self.pose.Y())}")
        logger.debug(f"Gyro Degree: {self.get_gyro_heading_degrees()}")
        logger.debug(f"Robot Heading: {self.pose.rotation().degrees()}")

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

        # if maybe_result is not None:
        #     SmartDashboard.putNumberArray("Limelight botpose", maybe_result.botpose)
        #     SmartDashboard.putNumber("Limelight timestamp", maybe_result.timestamp)

        SmartDashboard.putNumber("Heartbeat", self.heartbeat)
        SmartDashboard.putNumber("Robot X", self.pose.X())
        SmartDashboard.putNumber("Robot Y", self.pose.Y())
        SmartDashboard.putNumber("Robot Heading", self.pose.rotation().degrees())
        SmartDashboard.putNumber("field_sim X", self.field_sim.getRobotPose().X())

    def _odometry_periodic(self):
        # Update the odometry
        positions = [module.get_position() for module in self.modules]
        robot_rotation = self.get_gyro_heading_rotation2d()
        current_pos = self.odometry.getEstimatedPosition()
        self.odometry.update(robot_rotation, tuple(positions))
        estimated_pos = self.odometry.getEstimatedPosition()
        return current_pos, estimated_pos

    def simulationPeriodic(self):
        """
        Update simulation model.
        This should be called during simulationPeriodic in the robot class.
        """
        # Calculate time delta
        current_time = wpilib.Timer.getFPGATimestamp()
        dt = current_time - self.prev_sim_time
        if dt <= 0:
            dt = 0.02  # Use default timestep on first call
        self.prev_sim_time = current_time

        # Get the current module states
        module_states = [module.get_state() for module in self.modules]

        # Convert to chassis speeds
        chassis_speeds = self.kinematics.toChassisSpeeds(module_states)

        # Get robot-relative speeds
        vx = chassis_speeds.vx  # m/s
        vy = chassis_speeds.vy  # m/s
        omega = chassis_speeds.omega  # rad/s

        # Convert to field-relative movement
        heading = self.sim_pose.rotation().radians()
        dx = (vx * math.cos(heading) - vy * math.sin(heading)) * dt
        dy = (vx * math.sin(heading) + vy * math.cos(heading)) * dt
        dtheta = omega * dt

        # Update simulated robot pose
        new_x = self.sim_pose.X() + dx
        new_y = self.sim_pose.Y() + dy
        new_theta = self.sim_pose.rotation().radians() + dtheta

        self.sim_pose = Pose2d(new_x, new_y, Rotation2d(new_theta))

        # Update simulated gyro
        if hasattr(self.gyro, 'set_yaw'):
            degrees_val = self.sim_pose.rotation().degrees()
            self.gyro.set_yaw(degrees_val)

        # Update module visualizations
        self._update_module_visualizations()

        # Update odometry with simulated values
        module_positions = [module.get_position() for module in self.modules]
        self.odometry.update(self.sim_pose.rotation(), tuple(module_positions))

    def _update_module_visualizations(self):
        """
        Update the visualization of swerve modules on the field.
        """
        if not wpilib.RobotBase.isSimulation():
            return

        for i, module in enumerate(self.modules):
            # Get module position relative to robot center
            module_translation = self._get_module_translations()[i]
            x_offset = module_translation.X()  # Already in meters
            y_offset = module_translation.Y()  # Already in meters

            # Calculate module position in field coordinates
            robot_heading = self.sim_pose.rotation().radians()
            rotated_x = x_offset * math.cos(robot_heading) - y_offset * math.sin(robot_heading)
            rotated_y = x_offset * math.sin(robot_heading) + y_offset * math.cos(robot_heading)

            module_x = self.sim_pose.X() + rotated_x
            module_y = self.sim_pose.Y() + rotated_y

            # Get the module's wheel direction
            module_state = module.get_state()
            module_angle = self.sim_pose.rotation().rotateBy(module_state.angle)

            # Update the module pose
            self.module_poses[i] = Pose2d(module_x, module_y, module_angle)

        # Update field visualization
        self.module_visualization.setPoses(self.module_poses)

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
        :param y_speed_inches_per_second:    Speed to the left (from the driver's perspective)
        :param rot_speed_degrees_per_second: Desired rotational speed, CCW is positive.
        """
        desaturated_module_states = self._speeds_to_states(
            x_speed_inches_per_second,
            y_speed_inches_per_second,
            rot_speed_degrees_per_second,
        )
        for module, state in zip(self.modules, desaturated_module_states):
            module.set_desired_state(state)

    # --------------------------------------
    # Private methods to compute module states
    # --------------------------------------

    # def _limelight_periodic(self) -> Optional[limelightresults.GeneralResult]:
    #     if self.limelight is None:
    #         return None
    #     else:
    #         # Several results available. See https://docs.limelightvision.io/docs/docs-limelight/apis/limelightlib-python#websocket-based
    #         generalResult = self.limelight.get_results()
    #         if generalResult is None:
    #             return None
    #         else:
    #             return generalResult


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
        )
        swerve_module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        desaturated_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states,
            inchesToMeters(DriveConstants.MAX_SPEED_INCHES_PER_SECOND),
        )
        return desaturated_module_states

    def _get_chassis_speeds(
            self,
            x_speed_inches_per_second: inches_per_second,
            y_speed_inches_per_second: inches_per_second,
            rot_speed_degrees_per_second: degrees_per_second,
    ) -> ChassisSpeeds:
        # ChassisSpeeds expects meters and radians
        x_speed_meters_per_second = inchesToMeters(x_speed_inches_per_second)
        y_speed_meters_per_second = inchesToMeters(y_speed_inches_per_second)
        rot_speed_radians = degreesToRadians(rot_speed_degrees_per_second)

        cs = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed_meters_per_second,
            y_speed_meters_per_second,
            rot_speed_radians,
            -self.get_gyro_heading_rotation2d(),
        )
        return cs

    # --------------------------------------
    # Private methods for initialization
    # --------------------------------------

    def _initialize_odometry(
            self, kinematics: SwerveDrive4Kinematics, initial_pose : Optional[Pose2d] = None
    ) -> SwerveDrive4PoseEstimator:
        return SwerveDrive4PoseEstimator(
            kinematics=kinematics,
            gyroAngle=self.get_gyro_heading_rotation2d(),
            modulePositions=[module.get_position() for module in self.modules],
            initialPose=initial_pose or Pose2d(x = 0.0, y = 0.0, rotation = self.get_gyro_heading_rotation2d()),
            stateStdDevs=np.array([0.1, 0.1, 0.1]),  # X, Y, rotation standard deviations
            visionMeasurementStdDevs=np.array([0.7, 0.7, 9999999])  # Vision measurement uncertainties (values from "Using WPILib's Pose Estimator" sample at https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2)
        )

    def _get_module_translations(self) -> list[wpimath.geometry.Translation2d]:
        """
        Returns the physical positions of each swerve module relative to the center of the robot.
        The order should match the order of modules in self.modules:
        [FrontRight, FrontLeft, BackLeft, BackRight]

        Returns:
            list[Translation2d]: List of module positions in meters
        """
        # Create Translation2d objects for each module position
        # The coordinate system is:
        # - Positive x is forward
        # - Positive y is left
        # - Origin (0,0) is at robot center
        translations = [
            wpimath.geometry.Translation2d(
                inchesToMeters(DriveConstants.WHEELBASE_HALF_LENGTH),
                inchesToMeters(-DriveConstants.TRACK_HALF_WIDTH)
            ),  # Front Right
            wpimath.geometry.Translation2d(
                inchesToMeters(DriveConstants.WHEELBASE_HALF_LENGTH),
                inchesToMeters(DriveConstants.TRACK_HALF_WIDTH)
            ),  # Front Left
            wpimath.geometry.Translation2d(
                inchesToMeters(-DriveConstants.WHEELBASE_HALF_LENGTH),
                inchesToMeters(DriveConstants.TRACK_HALF_WIDTH)
            ),  # Back Left
            wpimath.geometry.Translation2d(
                inchesToMeters(-DriveConstants.WHEELBASE_HALF_LENGTH),
                inchesToMeters(-DriveConstants.TRACK_HALF_WIDTH)
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
        pid_output_x = metersToInches(self.x_controller.calculate(present_x))
        clamped_x = clamp(
            val = pid_output_x,
            min_val = -DriveConstants.MAX_SPEED_INCHES_PER_SECOND,
            max_val = DriveConstants.MAX_SPEED_INCHES_PER_SECOND,
        )  # Drive expects inches per second.

        present_y = self.pose.Y()
        pid_output_y = metersToInches(self.y_controller.calculate(present_y))
        clamped_y = clamp(
            val = pid_output_y,
            min_val = -DriveConstants.MAX_SPEED_INCHES_PER_SECOND,
            max_val = DriveConstants.MAX_SPEED_INCHES_PER_SECOND,
        )

        present_rot = self.pose.rotation().degrees()
        present_rot = wpimath.inputModulus(present_rot, -180, 180)
        pid_output_rot = self.rot_controller.calculate(present_rot)
        clamped_rot = clamp(
            val = pid_output_rot,
            min_val = -DriveConstants.MAX_DEGREES_PER_SECOND,
            max_val = DriveConstants.MAX_DEGREES_PER_SECOND,
        )

        # Send the values to the drive train.
        self.drive(x_speed_inches_per_second=clamped_x, y_speed_inches_per_second=clamped_y, rot_speed_degrees_per_second=clamped_rot)

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

    @staticmethod
    def _initialize_pid_controllers() -> (
            tuple[ProfiledPIDController, ProfiledPIDController, ProfiledPIDController]
    ):
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
            DriveConstants.PIDX_KD,
            TrapezoidProfile.Constraints(
                inchesToMeters(DriveConstants.HORIZ_MAX_V), inchesToMeters(DriveConstants.HORIZ_MAX_A)
            ),
        )
        x_controller.setTolerance(
            inchesToMeters(DriveConstants.HORIZ_POS_TOL), inchesToMeters(DriveConstants.HORIZ_VEL_TOL)
        )

        # Controller for the y direction (+ left, viewed by the driver)
        y_controller = ProfiledPIDController(
            DriveConstants.PIDY_KP,
            0,
            DriveConstants.PIDY_KD,
            TrapezoidProfile.Constraints(
                inchesToMeters(DriveConstants.HORIZ_MAX_V), inchesToMeters(DriveConstants.HORIZ_MAX_A)
            ),
        )
        y_controller.setTolerance(
            inchesToMeters(DriveConstants.HORIZ_POS_TOL), inchesToMeters(DriveConstants.HORIZ_VEL_TOL)
        )

        # Controller for the rotation direction (+ counterclockwise, viewed from above)
        # Continuous input enabled so that we can turn left or right, whichever is shorter.
        rot_controller = ProfiledPIDController(
            DriveConstants.PID_ROT_KP,
            0,
            DriveConstants.PID_ROT_KD,
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

    # def _on_new_vision_result(self, result: limelightresults.GeneralResult):
    #     # Based on "Using WPILib's Pose Estimator" at https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
    #     # Odd that we used both odometry and gyro to get the yaw, but that's from limelight sample
    #     estimated_yaw = self.odometry.getEstimatedPosition().rotation().degrees()
    #     self.limelight.update_robot_orientation(estimated_yaw, 0, 0, 0, 0, 0)
    #     megatag2_estimate = result.botpose_wpiblue

    #     # If our angular velocity is > 360 degrees per second, ignore vision updates
    #     angular_velocity = self.gyro.get_yaw_rate().value
    #     if abs(angular_velocity) > 360:
    #         reject_update = True
    #     # If we didn't actually see any tags, ignore vision updates
    #     if megatag2_estimate.tagCount == 0:
    #         reject_update = True
    #     if not reject_update:
    #         self.odometry.addVisionMeasurement(megatag2_estimate, result.timestamp)

def clamp(val, min_val, max_val):
    """Returns a number clamped to minval and maxval."""
    return max(min(val, max_val), min_val)