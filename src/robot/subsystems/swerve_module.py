from phoenix6.controls import PositionVoltage
from phoenix6.hardware import CANcoder
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals import InvertedValue, NeutralModeValue

class SwerveModule:
    def __init__(self, drive_motor_bus_id, turn_motor_bus_id, cancoder_bus_id):
        self.drive_motor = TalonFX(drive_motor_bus_id)
        self.turn_motor = TalonFX(turn_motor_bus_id)
        self.can_coder = CANcoder(cancoder_bus_id)  # ID number

        self.turn_motor.configurator.apply(self._configure_turn_motor())
        self.drive_motor.configurator.apply(self._configure_drive_motor())
        self.can_coder.configurator.apply(self._configure_cancoder())

    def set_drive_speed(self, speed):
        self.drive_motor.set(speed)

    def set_turn_angle(self, angle_degrees):
        # Position request starts at position 0, but can be modified later.
        position_request = PositionVoltage(0).with_slot(0)
        self.turn_motor.set(position_request) # ???
        raise NotImplementedError("set_turn_angle not implemented yet")

    def get_turn_angle(self):
        return self.turn_motor.get_position()

    def get_location(self):
        self.get_position().translation()

    def _configure_turn_motor(self):
        configuration = TalonFXConfiguration()

        configuration.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        configuration.motor_output.neutral_mode = NeutralModeValue.COAST

        # Set control loop parameters for "slot 0", the profile we'll use for position control.
        configuration.slot0.k_p = 1.0  # An error of one rotation results in 1.0V to the motor.
        configuration.slot0.k_i = 0.0  # No integral control
        configuration.slot0.k_d = 0.0  # No differential component
        # Voltage control mode peak outputs.  I'm only using a reduced voltage
        # for this test because it is an unloaded and barely secured motor.
        # Ordinarily, we would not change the default value, which is 16V.
        configuration.voltage.peak_forward_voltage = 6  # Peak output voltage of 6V.
        configuration.voltage.peak_reverse_voltage = -6  # And likewise for reverse.

        return configuration

    def _configure_drive_motor(self):
        configuration = TalonFXConfiguration()

        configuration.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        configuration.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # Set control loop parameters for "slot 0", the profile we'll use for position control.
        configuration.slot0.k_p = 1.0  # An error of one rotation results in 1.0V to the motor.
        configuration.slot0.k_i = 0.0  # No integral control
        configuration.slot0.k_d = 0.0  # No differential component
        # Voltage control mode peak outputs.  I'm only using a reduced voltage
        # for this test because it is an unloaded and barely secured motor.
        # Ordinarily, we would not change the default value, which is 16V.
        configuration.voltage.peak_forward_voltage = 6  # Peak output voltage of 6V.
        configuration.voltage.peak_reverse_voltage = -6  # And likewise for reverse.

        return configuration


    def _configure_cancoder(self):
        configuration = CANcoderConfiguration()
        configuration.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        configuration.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # Set control loop parameters for "slot 0", the profile we'll use for position control.
        configuration.slot0.k_p = 1.0  # An error of one rotation results in 1.0V to the motor.
        configuration.slot0.k_i = 0.0  # No integral control
        configuration.slot0.k_d = 0.0  # No differential component
        # Voltage control mode peak outputs.  I'm only using a reduced voltage
        # for this test because it is an unloaded and barely secured motor.
        # Ordinarily, we would not change the default value, which is 16V.
        configuration.voltage.peak_forward_voltage = 6  # Peak output voltage of 6V.
        configuration.voltage.peak_reverse_voltage = -6  # And likewise for reverse.

        return configuration
