from wpilib import PWMMotorController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState


class SwerveModule:
    def __init__(self, driveMotorId: int, turnMotorId: int) -> None:
        """
        Initialize the swerve module.

        Parameters:
        - drive_motor_id: int - Motor controller ID for driving.
        - turn_motor_id: int - Motor controller ID for turning.
        """
        self.driveMotor = PWMMotorController("driveMotor", driveMotorId)
        self.turnMotor = PWMMotorController("turnMotor", turnMotorId)
        self.currentState = SwerveModuleState()

    def setDesiredState(self, state: SwerveModuleState) -> None:
        """
        Set the desired state for the swerve module.

        Parameters:
        - state: SwerveModuleState - Desired speed and angle.
        """
        # Optimize module state for minimal turning
        optimized_state = SwerveModuleState.optimize(state, self.getCurrentAngle())

        # Set motor speeds
        self.driveMotor.set(optimized_state.speed)
        self.turnMotor.setAngle(optimized_state.angle)

        # Update current state
        self.currentState = optimized_state

    def getState(self) -> SwerveModuleState:
        """
        Get the current state of the swerve module.

        Returns:
        - SwerveModuleState: Current speed and angle of the module.
        """
        return self.currentState

    def getCurrentAngle(self) -> Rotation2d:
        """
        Get the current angle of the swerve module **in degrees**.

        Returns:
        - Rotation2d: The module's current angle in degrees.
        """
        return self.turnMotor.getAngle()

    def stop(self) -> None:
        """
        Stop the swerve module.
        """
        self.driveMotor.stop()
        self.turnMotor.stop()
