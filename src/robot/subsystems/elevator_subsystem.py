import commands2
import wpilib
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls import PositionVoltage
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import InvertedValue, NeutralModeValue, ReverseLimitValue
from constants.elevatorconstants import ElevatorConstants
from constants.new_types import inches
from util.current_threshold import CurrentThreshold


class ElevatorSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()  # Call the Subsystem class's (the "super" part) init.
        self.current_threshold = CurrentThreshold("Elevator Motor", ElevatorConstants.STOP_CURRENT)

        # ---------------------------------------------------------------------
        # Set up motors, their encoders, and the drivetrain.
        # ---------------------------------------------------------------------

        # Create the motor
        self.elevator_motor = TalonFX(ElevatorConstants.ELEVATOR_MOTOR)

        # Configure the motor.
        self.elevator_motor.configurator.apply(self._configure_elevator_motor())

        # Create lower limit switch
        self.lower_limit = wpilib.DigitalInput(1)
            
        # Create higher limit switch
        self.higher_limit = wpilib.DigitalInput(0)
        
        self.panic_stop = False

        self.initialized: bool = False
        if not self.lower_limit.get():
            self.elevator_motor.set_position(self._inches_to_motor_rot(ElevatorConstants.HOME))
            self.initialized = True



        # Position request starts at position 0, but can be modified later.
        self.position_request = PositionVoltage(0).with_slot(0)

        # Give an initial position in rotations we are trying to get to.
        self.goal_pos = self._inches_to_motor_rot(ElevatorConstants.HOME)


    ###########################################################################
    # Methods in base classes that we override here                           #
    ###########################################################################

    def periodic(self):
        """
        This method runs once every 20 msec in all modes (including simulation).
        """
        # Send data to the dashboard
        # motor_rotations = self.elevator_motor.get_position().value
        # height = self._motor_rot_to_inches(motor_rotations)
        # wpilib.SmartDashboard.putString(
        #     "DB/String 4", 'elev: {:5.2f} in."'.format(height)
        # )


        wpilib.SmartDashboard.putBoolean("At lowest height", self.lower_limit.get())
        wpilib.SmartDashboard.putBoolean("At highest height", self.higher_limit.get())

        motor_current = self.elevator_motor.get_stator_current().value
        
        self.current_threshold.is_exceeded(motor_current)

    def simulationPeriodic(self):
        """Called in simulation after periodic() to update simulation variables."""
        pass

    ###########################################################################
    # Methods to use in commands, either created in this class or elsewhere   #
    ###########################################################################

    def set_goal_height_inches(self, height: inches):
        """Set the goal in inches that the elevator drives toward"""
        # Convert because internally, we use rotations.
        self.goal_pos = self._inches_to_motor_rot(height)

    def lower_limit_reached(self):
        return not self.lower_limit.get()
    
    def higher_limit_reached(self):
        return not self.higher_limit.get()

    def get_current_height_inches(self) -> inches:
        """Get the current height of the elevator in inches"""
        return self._motor_rot_to_inches(self.elevator_motor.get_position().value)

    def move_to_goal(self):
        """Move toward the goal position"""
        if self.initialized:
            motor_duty = self.elevator_motor.get_duty_cycle().value

            if (self.lower_limit_reached() and motor_duty < -0.05) or (self.higher_limit_reached() and motor_duty > 0.05):
                print("Panick Stop!!")
                self.elevator_motor.set(0.0)
                self.panic_stop = True
            else:
                self.elevator_motor.set_control(
                    self.position_request.with_position(self.goal_pos)
                )
        else:
            # If not initialized, move downward slowly to find the bottom.
            self.elevator_motor.set(-0.1)
            if not self.lower_limit.get():
                self.elevator_motor.set(0.0)
                rotations = self._inches_to_motor_rot(ElevatorConstants.HOME)
                self.elevator_motor.set_position(rotations, timeout_seconds=10.0)
                self.initialized = True

    def initialized_and_stop(self):
        if self.initialized:
            self.elevator_stop()
            
        else:
            # If not initialized, move downward slowly to find the bottom.
            self.elevator_motor.set(-0.1)
            if not self.lower_limit.get():
                self.elevator_motor.set(0.0)
                rotations = self._inches_to_motor_rot(ElevatorConstants.HOME)
                self.elevator_motor.set_position(rotations, timeout_seconds=10.0)
                self.initialized = True
        

    # def initialize_bottom_limit(self):
    #     initialized: bool = False
    #     # Initialize if the bottom limit exists (?)
    #     if (
    #         self.elevator_motor.get_reverse_limit().value
    #         == ReverseLimitValue.CLOSED_TO_GROUND
    #     ):
    #         rotations = self._inches_to_motor_rot(ElevatorConstants.HOME)
    #         self.elevator_motor.set_position(rotations)
    #         initialized = True
    #     return initialized

    def is_at_goal(self):
        return self.panic_stop
    
    def elevator_up(self):
        self.elevator_motor.set(0.2)
    
    def elevator_down(self):
        self.elevator_motor.set(-0.2)

    def elevator_stop(self):
        self.elevator_motor.set(0.0)
        


    ###########################################################################
    # Utility methods to use in this class                                    #
    ###########################################################################

    @staticmethod
    def _motor_rot_to_inches(rot: float) -> inches:
        """Convert motor shaft rotations to height in inches."""
        return (
            rot * ElevatorConstants.SCREW_INCHES_PER_ROT / ElevatorConstants.GEAR_RATIO
            + ElevatorConstants.HEIGHT_OFFSET
        )

    @staticmethod
    def _inches_to_motor_rot(height: inches) -> float:
        """Convert height to motor shaft position in rotations."""
        return (
            (height - ElevatorConstants.HEIGHT_OFFSET)
            / ElevatorConstants.SCREW_INCHES_PER_ROT
            * ElevatorConstants.GEAR_RATIO
        )

    @staticmethod
    def _configure_elevator_motor() -> TalonFXConfiguration:
        configuration = TalonFXConfiguration()

        configuration.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        configuration.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # Set control loop parameters for "slot 0", the profile we'll use for position control.
        configuration.slot0.k_p = (
            0.4  # An error of one rotation results in 1.0V to the motor.
        )
        configuration.slot0.k_i = 0.01  # No integral control
        configuration.slot0.k_d = 0.0  # No differential component

        return configuration
    