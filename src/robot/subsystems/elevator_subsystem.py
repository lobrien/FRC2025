import commands2
import wpilib
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls import PositionVoltage
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import InvertedValue, NeutralModeValue
from constants.elevatorconstants import ElevatorConstants


class ElevatorSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()  # Call the Subsystem class's (the "super" part) init.

        # ---------------------------------------------------------------------
        # Set up motors, their encoders, and the drivetrain.
        # ---------------------------------------------------------------------

        # Create the motor
        self.elevator_motor = TalonFX(ElevatorConstants.ELEVATOR_MOTOR)

        # Apply it to the motor.
        self.elevator_motor.configurator.apply(self._configure_elevator_motor())

        # LOB: Speculative!
        self.bottom_limit = self.elevator_motor.get_reverse_limit()
        # self.bottom_limit = (
        #     self.elevator_motor.getReverseLimitSwitch()
        # )  # Plan for having 2 limit switch. One at the bottom and one at the highest height we want to go to

        # Position request starts at position 0, but can be modified later.
        self.position_request = PositionVoltage(0).with_slot(0)

        # Give an initial position in rotations we are trying to get to.
        self.goal_pos = self._inches_to_motor_rot(ElevatorConstants.HOME)

        # Make sure we initialize the encoder properly.
        self.initialized = self.initialize_bottom_limit()

    ###########################################################################
    # Methods in base classes that we override here                           #
    ###########################################################################

    def periodic(self):
        """
        This method runs once every 20 msec in all modes (including simulation).
        """
        # Send data to the dashboard
        motor_rotations = self.elevator_motor.get_position().value
        height = self._motor_rot_to_inches(motor_rotations)
        wpilib.SmartDashboard.putString(
            "DB/String 4", 'elev: {:5.2f} in."'.format(height)
        )

    def simulationPeriodic(self):
        """Called in simulation after periodic() to update simulation variables."""
        pass

    ###########################################################################
    # Methods to use in commands, either created in this class or elsewhere   #
    ###########################################################################

    def set_goal_height_inches(self, height: float):
        """Set the goal in inches that the elevator drives toward"""
        # Convert because internally, we use rotations.
        self.goal_pos = self._inches_to_motor_rot(height)

    def get_current_height_inches(self) -> float:
        """Get the current height of the elevator in inches"""
        return self._motor_rot_to_inches(self.elevator_motor.get_position().value)

    def move_to_goal(self):
        """Move toward the goal position"""
        if self.initialized:
            self.elevator_motor.set_control(
                self.position_request.with_position(self.goal_pos)
            )
        else:
            # If not initialized, move downward slowly to find the bottom.
            self.elevator_motor.set(-0.1)
            if self.bottom_limit.get():
                self.elevator_motor.set(0.0)
                rotations = self._inches_to_motor_rot(ElevatorConstants.HOME)
                self.elevator_motor.set_position(rotations, timeout_seconds=10.0)
                self.initialized = True

    def is_at_goal(self) -> bool:
        return False  # Never end unless interrupted. LOB: ???

    # LOB: Speculative! TODO: Check this code
    def initialize_bottom_limit(self):
        initialized: bool = False
        # Initialize if the bottom limit exists (?)
        if self.elevator_motor.get_reverse_limit().value:
            rotations = self._inches_to_motor_rot(ElevatorConstants.HOME)
            self.elevator_motor.set_position(rotations)
            initialized = True
        return initialized

    ###########################################################################
    # Utility methods to use in this class                                    #
    ###########################################################################

    @staticmethod
    def _motor_rot_to_inches(rot: float) -> float:
        """Convert motor shaft rotations to height in inches."""
        return (
            rot
            * ElevatorConstants.SPROCKET_CIRC
            * ElevatorConstants.RIG
            / ElevatorConstants.GEAR_RATIO
            + ElevatorConstants.HEIGHT_OFFSET
        )

    @staticmethod
    def _inches_to_motor_rot(height: float) -> float:
        """Convert height to motor shaft position in rotations."""
        return (
            (height - ElevatorConstants.HEIGHT_OFFSET)
            * ElevatorConstants.GEAR_RATIO
            / ElevatorConstants.SPROCKET_CIRC
            / ElevatorConstants.RIG
        )

    @staticmethod
    def _configure_elevator_motor() -> TalonFXConfiguration:
        configuration = TalonFXConfiguration()

        configuration.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        configuration.motor_output.neutral_mode = NeutralModeValue.COAST

        # Set control loop parameters for "slot 0", the profile we'll use for position control.
        configuration.slot0.k_p = (
            1.0  # An error of one rotation results in 1.0V to the motor.
        )
        configuration.slot0.k_i = 0.0  # No integral control
        configuration.slot0.k_d = 0.0  # No differential component

        return configuration
