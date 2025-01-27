from phoenix6.hardware.talonfx import TalonFX
from phoenix6.configs import TalonFXConfiguration
import phoenix6.signals
from phoenix6 import PositionVoltage

class SwerveModule:
    def __init__(self, drive_motor_bus_id, turn_motor_bus_id):
        self.drive_motor = TalonFX(drive_motor_bus_id)
        self.turn_motor = TalonFX(turn_motor_bus_id)

        self.configuration = TalonFXConfiguration()

        self.configuration.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.configuration.motor_output.neutral_mode = NeutralModeValue.COAST

        # Set control loop parameters for "slot 0", the profile we'll use for position control.
        self.configuration.slot0.k_p = 1.0  # An error of one rotation results in 1.0V to the motor.
        self.configuration.slot0.k_i = 0.0  # No integral control
        self.configuration.slot0.k_d = 0.0  # No differential component
        # Voltage control mode peak outputs.  I'm only using a reduced voltage
        # for this test because it is an unloaded and barely secured motor.
        # Ordinarily, we would not change the default value, which is 16V.
        self.configuration.voltage.peak_forward_voltage = 6  # Peak output voltage of 6V.
        self.configuration.voltage.peak_reverse_voltage = -6  # And likewise for reverse.

        # Set control loop parameters for slot 1, which we'll use with motion magic position control
        self.configuration.slot1.k_p = 1.0  # An error of one rotation results in 1.0V to the motor.
        self.configuration.slot1.k_i = 0.0  # No integral control
        self.configuration.slot1.k_d = 0.0  # No differential component
        # And set the motion magic parameters.
        self.configuration.motion_magic.motion_magic_cruise_velocity = 1  # 1 rotation/sec
        self.configuration.motion_magic.motion_magic_acceleration = 1  # Take approximately 1 sec (vel/accel) to get to full speed
        self.configuration.motion_magic.motion_magic_jerk = 10  # Take approx. 0.1 sec (accel/jerk) to reach max accel.

        self.drive_motor.configurator.apply(self.configuration)
        self.turn_motor.configurator.apply(self.configuration)

    def set_drive_speed(self, speed):
        self.drive_motor.set(speed)

    def set_turn_angle(self, angle_degrees):
        # Position request starts at position 0, but can be modified later.
        position_request = phoenix6.controls.PositionVoltage(0).with_slot(0)
        self.turn_motor.set(position_request) # ???
        raise NotImplementedError("set_turn_angle not implemented yet")

    def get_turn_angle(self):
        return self.turn_motor.get_position()

