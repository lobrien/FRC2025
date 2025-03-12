import math

import numpy as np
import wpimath
from phoenix6.controls import PositionVoltage, NeutralOut
from phoenix6.hardware import CANcoder
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpilib import RobotBase
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.units import degrees, meters, inches, meters_per_second, degreesToRadians
from wpimath.units import (
    metersToInches,
    inchesToMeters,
    degreesToRotations,
    rotationsToDegrees,
)

import wpilib

from constants.driveconstants import DriveConstants
from constants.new_types import percentage, inches_per_second, degrees_per_second


def _calc_drive_effort(speed: inches_per_second) -> percentage:
    drive_effort = speed / DriveConstants.MAX_SPEED_INCHES_PER_SECOND
    drive_effort_clamped = max(min(drive_effort, 1.0), -1.0)
    return percentage(drive_effort_clamped)


class SwerveModule:
    def __init__(
            self,
            name: str,
            drive_motor_bus_id: int,
            turn_motor_bus_id: int,
            cancoder_bus_id: int,
            offset_rotations: float,
            offset_translation: tuple[inches, inches]
    ) -> None:
        """
        Initialize a swerve module.

        Args:
            name: Name identifier for the module
            drive_motor_bus_id: CAN ID of the drive motor
            turn_motor_bus_id: CAN ID of the turn motor
            cancoder_bus_id: CAN ID of the CANcoder
            offset_rotations: Offset for the CANcoder, range [-1,1] in rotations
            offset_translation: Position of this module relative to the robot center
        """
        # Needed for outputting to NetworkTables in periodic() fn
        self.name = name
        self.drive_motor = TalonFX(drive_motor_bus_id)
        self.turn_motor = TalonFX(turn_motor_bus_id)
        self.can_coder = CANcoder(cancoder_bus_id)
        self.offset_translation = offset_translation

        self.turn_motor.configurator.apply(self._configure_turn_motor())
        self.drive_motor.configurator.apply(self._configure_drive_motor())

        # Create control requests for the motors.
        # Either brake or coast, depending on motor configuration.
        self.brake_request = NeutralOut()

        # Position request starts at position 0, but can be modified later.
        self.position_request = PositionVoltage(0).with_slot(0)

        # Initialize encoders.
        # The drive encoder just starts at zero, this one is easy.
        self.drive_motor.set_position(0.0)

        # CANCoder: absolute encoder that reads steering angle.
        # rotation_offset is the magnet offset, read by the CANcoder when steering is aligned at physical zero.
        self.rotation_offset_degrees = rotationsToDegrees(offset_rotations)

        # The turn motor also has an encoder, and we use the firmware PID
        # to set the turn position. Initialize based on CANcoder's position.
        steering_position_rotations = self._get_can_coder_pos_normalized()
        self.turn_motor.set_position(
            steering_position_rotations * DriveConstants.TURN_GEAR_RATIO
        )

        # Simulation properties
        self.sim_drive_position = inches(0.0)       # Simulated drive position in inches
        self.sim_turn_position = degrees(0.0)       # Simulated turn position in degrees
        self.sim_drive_velocity = inches_per_second(0.0)  # Simulated velocity in inches/sec
        self.sim_turn_velocity = degrees_per_second(0.0)  # Simulated velocity in degrees/sec
        self.sim_desired_state = SwerveModuleState(0, Rotation2d(0))
        self.prev_sim_time = 0.0

    # --------------------------------------
    # Public methods
    # --------------------------------------

    # Sets the drive to the given speed, expressed as a percentage of full speed (range -1 to 1).
    def set_drive_effort(self, speed_pct: percentage):
        self.drive_motor.set(speed_pct)

        # Update simulation values
        if RobotBase.isSimulation():
            # Assuming 12V system - convert effort to velocity
            max_speed_ips = DriveConstants.MAX_SPEED_INCHES_PER_SECOND
            self.sim_drive_velocity = speed_pct * max_speed_ips

    # Sets the turn to the given speed, expressed as a percentage of full speed (range -1 to 1).
    def set_turn_effort(self, speed_pct: percentage):
        self.turn_motor.set(speed_pct)

        # Update simulation values
        if RobotBase.isSimulation():
            # Assuming 12V system - convert effort to angular velocity
            max_speed_dps = DriveConstants.MAX_DEGREES_PER_SECOND
            self.sim_turn_velocity = speed_pct * max_speed_dps

    def set_turn_angle(self, angle_degrees: degrees):
        steering_rotation = degreesToRotations(angle_degrees)

        # Command the motor to the desired position
        self.turn_motor.set_control(
            self.position_request.with_position(
                steering_rotation * DriveConstants.TURN_GEAR_RATIO
            )
        )

        # Update simulation values
        if RobotBase.isSimulation():
            self.sim_desired_state = SwerveModuleState(
                self.sim_desired_state.speed,
                Rotation2d.fromDegrees(angle_degrees)
            )

    def get_turn_angle_degrees(self) -> degrees:
        """
        Returns the current turn angle of the module in degrees.
        Returns:
            Turn angle normalized to the range [-180,180].
        """
        if RobotBase.isSimulation():
            return self.sim_turn_position
        else:
            normalized_rotations = self._get_can_coder_pos_normalized()  # Range from 0 to 1
            _degrees = normalized_rotations * 360
            # Convert to [-180,180] range
            if _degrees > 180:
                _degrees = _degrees - 360
            return _degrees

    def get_state(self) -> SwerveModuleState:
        """
        Get the current state of the module
        Returns:
            Current state (speed and angle)
        """
        if RobotBase.isSimulation():
            speed_mps = inchesToMeters(self.sim_drive_velocity)
            angle = Rotation2d.fromDegrees(self.sim_turn_position)
        else:
            # Get actual wheel speed from encoder
            wheel_rps = (
                    self.drive_motor.get_velocity().value / DriveConstants.DRIVE_GEAR_RATIO
            )  # wheel rotations per second.
            speed_mps: meters_per_second = inchesToMeters(
                self._inches_per_rotation() * wheel_rps
            )
            # Get actual angle from encoder
            angle = Rotation2d.fromDegrees(self.get_turn_angle_degrees())

        return SwerveModuleState(speed_mps, angle)

    def get_position(self) -> SwerveModulePosition:
        """
        Returns the current position of the swerve module.

        This combines the current distance traveled by the drive motor (in meters)
        and the current angle of the turn motor (as a Rotation2d).

        Note: While we track distances in inches internally, WPILib's SwerveModulePosition
        expects the distance in meters, so we convert before returning.

        Returns:
            SwerveModulePosition: The current position of this swerve module
        """
        drive_position_meters = inchesToMeters(self._drive_position)
        turn_angle = Rotation2d.fromDegrees(self._turn_position)

        return SwerveModulePosition(drive_position_meters, turn_angle)

    @property
    def _drive_position(self) -> inches:
        """
        Get the current drive encoder position in inches.
        """
        if RobotBase.isSimulation():
            return self.sim_drive_position
        else:
            raw_position = self.drive_motor.get_position().value
            wheel_circumference = math.pi * DriveConstants.WHEEL_DIA
            gear_ratio = DriveConstants.DRIVE_GEAR_RATIO
            return raw_position / gear_ratio * wheel_circumference

    @property
    def _turn_position(self) -> degrees:
        """
        Get the current turn encoder position in degrees.
        """
        if RobotBase.isSimulation():
            return self.sim_turn_position
        else:
            # First try to use CANcoder if available
            if hasattr(self, 'can_coder') and self.can_coder is not None:
                return self.get_turn_angle_degrees()
            else:
                # Fall back to motor encoder
                raw_position = self.turn_motor.get_position().value
                gear_ratio = DriveConstants.TURN_GEAR_RATIO
                position_degrees = raw_position / gear_ratio * 360.0
                # Apply offset and normalize
                calibrated_position = position_degrees - self.rotation_offset_degrees
                normalized_position = wpimath.inputModulus(calibrated_position, -180.0, 180.0)
                return normalized_position

    def periodic(self):
        """
        Reports module data to dashboards and updates simulation.
        """
        # Update simulation if needed
        if RobotBase.isSimulation():
            self._simulation_periodic()

        # Dashboard updates
        wpilib.SmartDashboard.putString(
            f"{self.name}_turn_degrees",
            "degrees: {:5.1f}".format(self._get_full_turn_angle_from_motor()),
        )
        wpilib.SmartDashboard.putString(
            f"{self.name}_can_coder_pos_rotations",
            "rotations: {:5.3f}".format(self._get_can_coder_pos_normalized()),
        )

    def set_desired_state(self, desired_state: SwerveModuleState) -> None:
        """
        Command the module to the desired state.

        Args:
            desired_state: The SwerveModuleState to achieve
        """
        # Save state for simulation
        if RobotBase.isSimulation():
            self.sim_desired_state = desired_state
            optimized_state = desired_state
        else:
            # Optimize the state to minimize rotation
            current_degrees = self._get_full_turn_angle_from_motor()
            current_rotation = Rotation2d.fromDegrees(current_degrees)
            optimized_state = self._optimize(desired_state, current_rotation)

        # Command motors
        drive_effort = _calc_drive_effort(
            inches_per_second(metersToInches(optimized_state.speed))
        )
        wpilib.SmartDashboard.putString(
            f"{self.name}_drive_effort", "{:5.2f}".format(drive_effort)
        )
        geared_rotations = self._degrees_to_turn_count(optimized_state.angle.degrees())
        request = self.position_request.with_position(geared_rotations)

        self.set_drive_effort(drive_effort)
        self.turn_motor.set_control(request)

    def _simulation_periodic(self) -> None:
        """
        Update simulation model.
        """
        # Calculate time delta
        current_time = wpilib.Timer.getFPGATimestamp()
        dt = current_time - self.prev_sim_time
        if dt <= 0:
            dt = 0.02  # Use default timestep on first call
        self.prev_sim_time = current_time

        # Update simulated turn position (angle)
        desired_angle_degrees = self.sim_desired_state.angle.degrees()
        angle_diff = wpimath.inputModulus(
            desired_angle_degrees - self.sim_turn_position, -180.0, 180.0
        )

        # Simple model: move 90% of the way to target angle each cycle
        self.sim_turn_position += angle_diff * 0.9
        self.sim_turn_position = wpimath.inputModulus(self.sim_turn_position, -180.0, 180.0)

        # Update simulated drive position based on speed
        # Get speed in inches/second
        speed_inches_per_second = metersToInches(self.sim_desired_state.speed)

        # Update position based on speed and time delta
        self.sim_drive_position += speed_inches_per_second * dt

        # For velocity tracking
        self.sim_drive_velocity = speed_inches_per_second

    def stop(self):
        """Stop all motors on this module."""
        self.drive_motor.stopMotor()
        self.turn_motor.stopMotor()

        # Reset simulation values
        if RobotBase.isSimulation():
            self.sim_drive_velocity = inches_per_second(0.0)
            self.sim_turn_velocity = degrees_per_second(0.0)
            self.sim_desired_state = SwerveModuleState(0, Rotation2d.fromDegrees(self.sim_turn_position))

    # --------------------------------------
    # Private methods for configuration
    # --------------------------------------

    @staticmethod
    def _inches_per_rotation() -> inches:
        return DriveConstants.WHEEL_RADIUS * 2 * 3.14159

    @staticmethod
    def _configure_turn_motor() -> TalonFXConfiguration:
        configuration = TalonFXConfiguration()

        configuration.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        configuration.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # Set control loop parameters for "slot 0", the profile we'll use for position control.
        configuration.slot0.k_p = (
            1.0  # An error of one rotation results in 1.0V to the motor.
        )
        configuration.slot0.k_i = 0.0  # No integral control
        configuration.slot0.k_d = 0.0  # No differential component

        return configuration

    @staticmethod
    def _configure_drive_motor() -> TalonFXConfiguration:
        configuration = TalonFXConfiguration()

        configuration.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        configuration.motor_output.neutral_mode = NeutralModeValue.BRAKE

        return configuration

    # --------------------------------------
    # Private methods to help calculate desired state
    # --------------------------------------

    @staticmethod
    def _place_in_appropriate_0to360_scope(
            scope_reference_degrees: degrees, new_angle_degrees: degrees
    ) -> degrees:
        """
        Place the new_angle_degrees in the range that is a multiple of [0,360] that is closest
        to the scope_reference.
        """
        lower_offset = (
                scope_reference_degrees % 360
        )  # Modulo (remainder) is always positive when divisor (360) is positive.
        lower_bound = scope_reference_degrees - lower_offset
        upper_bound = lower_bound + 360

        # Adjust the new_angle_degrees to fit between the bounds.
        while new_angle_degrees < lower_bound:
            new_angle_degrees += 360
        while new_angle_degrees > upper_bound:
            new_angle_degrees -= 360

        # Adjust new_angle_degrees more to make sure it is within 180 degrees of the reference.
        if new_angle_degrees - scope_reference_degrees > 180:
            new_angle_degrees -= 360
        elif new_angle_degrees - scope_reference_degrees < -180:
            new_angle_degrees += 360

        return new_angle_degrees

    def _optimize(
            self, desired_state: SwerveModuleState, current_rotation: Rotation2d
    ) -> SwerveModuleState:
        """
        There are two ways for a swerve module to reach its goal:
        1) Rotate to its intended steering angle and drive at its intended speed.
        2) Rotate to the mirrored steering angle (subtract 180) and drive at the opposite of its intended speed.
        Optimizing finds the option that requires the smallest rotation.
        """
        target_angle = self._place_in_appropriate_0to360_scope(
            current_rotation.degrees(), desired_state.angle.degrees()
        )
        target_speed = desired_state.speed
        delta_degrees = target_angle - current_rotation.degrees()

        if abs(delta_degrees) > 90:
            optimized_target_speed = -target_speed
            if delta_degrees > 90:
                # Delta is positive
                optimized_target_angle = target_angle - 180
            else:
                # Delta is negative
                optimized_target_angle = target_angle + 180
        else:
            optimized_target_speed = target_speed
            optimized_target_angle = target_angle

        return SwerveModuleState(
            optimized_target_speed, Rotation2d.fromDegrees(optimized_target_angle)
        )

    # --------------------------------------
    # Private methods for turn (steering) angle
    # --------------------------------------

    @staticmethod
    def _degrees_to_turn_count(_degrees: degrees) -> float:
        """
        Converts between steering shaft angle in degrees and motor rotations.
        :param _degrees: turning angle in degrees
        Returns:
            Turn motor rotations
        """
        rotations = degreesToRotations(_degrees)
        return rotations * DriveConstants.TURN_GEAR_RATIO

    # Returns the CANCoder's current position as a percentage of full rotation (range [0,1]).
    def _get_can_coder_pos_normalized(
            self,
    ) -> (
            percentage
    ):  # the _ in front of a function is indicating that this is only should be used in this class NOT ANYWHERE ELSE
        can_coder_abs_pos = self.can_coder.get_absolute_position().value
        can_coder_offset = can_coder_abs_pos - degreesToRotations(
            self.rotation_offset_degrees
        )
        normalized = percentage(can_coder_offset % 1.0)
        return normalized

    def _get_turn_angle_from_motor(self) -> degrees:
        """
        Get the turn angle from the motor's encoder, normalized to the
        range [0.0, 360.0).  CCW looking from above is positive.
        Returns:
            Steering shaft angle in normalized degrees.
        """
        # TalonFX.get_position() returns a value in full rotations.
        motor_abs_rotations = self.turn_motor.get_position().value
        # Convert by gear ratio
        ratioed_rotations = motor_abs_rotations / DriveConstants.TURN_GEAR_RATIO
        # We're only interested in the fractional part
        steering_pct_rotated = ratioed_rotations % 1.0
        return rotationsToDegrees(steering_pct_rotated)

    def _get_full_turn_angle_from_motor(self) -> degrees:
        """
        Get the turn angle from the motor's encoder, without wrapping.
        In other words, if the steering has made two complete rotations
        CCW, the returned value will be 720.0 degrees.
        CCW looking from above is positive.
        Returns:
            Steering shaft angle in degrees.
        """
        # TalonFX.get_position() returns a value in full rotations.
        motor_abs_rotations = self.turn_motor.get_position().value
        # Convert by gear ratio
        ratioed_rotations = motor_abs_rotations / DriveConstants.TURN_GEAR_RATIO
        return rotationsToDegrees(ratioed_rotations)