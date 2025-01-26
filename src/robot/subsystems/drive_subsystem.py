import phoenix6
import commands2
import wpimath
import wpilib
import math


from constants.driveconstants import DriveConstants

# The `DriveSubsystem` class is a `Subsystem` that contains the robot's drive motors and sensors. It
# is responsible for moving the robot around on the field. Public methods exposed by this class
# should make logical sense for *any* kind of drive, whether it be tank, arcade, swerve, or hovercraft.
# For instance, you wouldn't want to expose a `setLeftSpeed` method for a swerve drive, because that
# doesn't make sense for a swerve drive. Instead, you would want to expose a `drive` method that takes
# a speed and a rotation, because that makes sense for *any* kind of drive.
class DriveSubsystem(commands2.Subsystem):  # Name what type of class this is
    def __init__(self):
        super().__init__()  # Allows the class to call parent class

        self.turn_motor = phoenix6.hardware.talon_fx.TalonFX(DriveConstants.TURN_BL)  # Configure motors
        self.drive_motor = phoenix6.hardware.talon_fx.TalonFX(DriveConstants.DRIVE_BL) # Motor's ID numbers

        self.can_coder = phoenix6.hardware.cancoder.CANcoder(DriveConstants.CAN_BL) #ID number

        self.configuration = phoenix6.configs.TalonFXConfiguration()

        self.configuration.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.configuration.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST

        self.can_coder.configurator.set_position(0)

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

    def drive(self, drive_speed:float, turn_speed:float) -> None:
        self.turn_motor.set(turn_speed)
        self.drive_motor.set(drive_speed)

    def _get_can_coder(self) -> float: # the _ in front of a function is indicating that this is only should be used in this class NOT ANYWHERE ELSE
        return self.can_coder.get_absolute_position().value   #the .value property doesn't seem to have () at the end
    
    def _get_wheel_degree(self) -> float:   #angle from -180 to 180 of the wheel
        can_coder_angle_pct = self._get_can_coder()
        angle_degrees = 180 * can_coder_angle_pct

        return angle_degrees

    def set_drive_angle(self, desired_angle_degrees):
        print(f"Setting angle to 45 from {self.turn_motor.get_position().value}")
        # convert degrees to range -1 to 1
        pct = desired_angle_degrees / 180.0

        ticks = 512
        request = self.position_request.with_position(ticks)
        sc = self.turn_motor.set_control(request)
        print(f"status code {sc}")
        # self.turn_motor.set_position(pct)

    # This periodic function is called every 20ms during the robotPeriodic phase
    # *in all modes*. It is called automatically by the Commands2 framework.
    def periodic(self):
        print(".", end="", flush=True)
        wpilib.SmartDashboard.putString('FR pos', 'rotations: {:5.1f}'.format(self.turn_motor.get_position().value)) 
        wpilib.SmartDashboard.putString('FR pos can coder', 'rotations: {:5.1f}'.format(self._get_can_coder()))
