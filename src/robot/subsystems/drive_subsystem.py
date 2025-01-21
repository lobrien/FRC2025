import phoenix6
import commands2
import wpimath
import wpilib

from wpimath import geometry

# DriveSubsystem Class
class DriveSubsystem(commands2.Subsystem):  # Name what type of class this is
    def __init__(self):
        # super().__init__()  # Allows the class to call parent class
        self.turn_motor = phoenix6.hardware.talon_fx.TalonFX(10)  # Configure motors
        self.drive_motor = phoenix6.hardware.talon_fx.TalonFX(12)  

        self.can_coder = phoenix6.hardware.cancoder.CANcoder(11)

        # self.drive_motor.set_position(0)

        self.configuration = phoenix6.configs.TalonFXConfiguration()

        self.configuration.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.configuration.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST

        # Configuration 
        self.drive_motor.configurator.apply(self.configuration)
        self.turn_motor.configurator.apply(self.configuration)

        # Set control loop parameters for "slot 0", the profile we'll use for position control.
        self.configuration.slot0.k_p = 1.0 # An error of one rotation results in 1.0V to the motor.
        self.configuration.slot0.k_i = 0.0 # No integral control
        self.configuration.slot0.k_d = 0.0 # No differential component
        # Voltage control mode peak outputs.  I'm only using a reduced voltage
        # for this test because it is an unloaded and barely secured motor.
        # Ordinarily, we would not change the default value, which is 16V.
        self.configuration.voltage.peak_forward_voltage = 6 # Peak output voltage of 6V.
        self.configuration.voltage.peak_reverse_voltage = -6 # And likewise for reverse.

        # Set control loop parameters for slot 1, which we'll use with motion magic position control
        self.configuration.slot1.k_p = 1.0 # An error of one rotation results in 1.0V to the motor.
        self.configuration.slot1.k_i = 0.0 # No integral control
        self.configuration.slot1.k_d = 0.0 # No differential component
        # And set the motion magic parameters.
        self.configuration.motion_magic.motion_magic_cruise_velocity = 1 # 1 rotation/sec
        self.configuration.motion_magic.motion_magic_acceleration = 1 # Take approximately 1 sec (vel/accel) to get to full speed
        self.configuration.motion_magic.motion_magic_jerk = 10 # Take approx. 0.1 sec (accel/jerk) to reach max accel.

        # Either brake or coast, depending on motor configuration; we chose brake above.
        self.brake_request = phoenix6.controls.NeutralOut()

        # Position request starts at position 0, but can be modified later.
        self.position_request = phoenix6.controls.PositionVoltage(0).with_slot(0)

        # A motion magic (MM) position request. MM smooths the acceleration.
        self.mm_pos_request = phoenix6.controls.MotionMagicVoltage(0).with_slot(1)


    def drive(self, drive_speed, turn_speed):
        self.turn_motor.set(turn_speed)
        self.drive_motor.set(drive_speed)

    def get_cancoder(self):
        return geometry.Rotation2d.fromRotations(self.can_coder.get_absolute_position().value())

    def periodic(self):
        wpilib.SmartDashboard.putString('FR pos', 'rotations: {:5.1f}'.format(self.drive_subsystem.turn_motor.get_position().value))
