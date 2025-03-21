import wpilib

# import rev
import phoenix6
from wpimath.units import seconds
import rev


class Myrobot(wpilib.TimedRobot):
    """Simple robot to drive motors for testing.
    Initially written for MAKO, but could easily be changed.
    """

    def __init__(self, period: seconds = 0.02):
        super().__init__(period)
        self.print_timer = None
        self.xbox = None
        self.kraken = None
        self.brake_request = None
        self.position_request = None
        self.mm_pos_request = None

    def robotInit(self):
        # A timer to help us print info periodically
        self.print_timer = wpilib.Timer()
        self.print_timer.start()

        # ----------------------------------------------------------------------
        # Human interface
        self.xbox = wpilib.XboxController(1)

        # ----------------------------------------------------------------------
        # Motor controllers

        # MAKO's left-side motor controllers, SparkMaxes
        self.motor2 = rev.SparkMax(33, rev.SparkMax.MotorType.kBrushless)
        # self.motor4 = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)

        # Test a Kraken
        # self.kraken = phoenix6.hardware.TalonFX(1)

        # Note that "set_position" does not _command_ a position, it tells
        # the Kraken that its encoder should read the given position at
        # its present location.
        # self.kraken.set_position(0)

        # ----------------------------------------------------------------------
        # Set up Kraken's configuration by first getting a default
        # # configuration object.
        # configuration = phoenix6.configs.TalonFXConfiguration()
        # # Motor direction and neutral mode
        # # Counterclockwise is positive when facing the motor shaft.
        # configuration.motor_output.inverted = (
        #     phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        # )
        # configuration.motor_output.neutral_mode = (
        #     phoenix6.signals.NeutralModeValue.BRAKE
        # )
        # # Set control loop parameters for "slot 0", the profile we'll use for position control.
        # configuration.slot0.k_p = (
        #     1.0  # An error of one rotation results in 1.0V to the motor.
        # )
        # configuration.slot0.k_i = 0.0  # No integral control
        # configuration.slot0.k_d = 0.0  # No differential component
        # # Voltage control mode peak outputs.  I'm only using a reduced voltage
        # # for this test because it is an unloaded and barely secured motor.
        # # Ordinarily, we would not change the default value, which is 16V.
        # configuration.voltage.peak_forward_voltage = 6  # Peak output voltage of 6V.
        # configuration.voltage.peak_reverse_voltage = -6  # And likewise for reverse.

        # # Set control loop parameters for slot 1, which we'll use with motion magic position control
        # configuration.slot1.k_p = (
        #     1.0  # An error of one rotation results in 1.0V to the motor.
        # )
        # configuration.slot1.k_i = 0.0  # No integral control
        # configuration.slot1.k_d = 0.0  # No differential component
        # # And set the motion magic parameters.
        # configuration.motion_magic.motion_magic_cruise_velocity = 1  # 1 rotation/sec
        # configuration.motion_magic.motion_magic_acceleration = (
        #     1  # Take approximately 1 sec (vel/accel) to get to full speed
        # )
        # configuration.motion_magic.motion_magic_jerk = (
        #     10  # Take approx. 0.1 sec (accel/jerk) to reach max accel.
        # )

        # ----------------------------------------------------------------------
        # Apply the configuration. Interestingly, CTRE's example has code that
        # attempts configuration 5 times.  This suggests that configuration
        # # doesn't always take, which is kind of dismaying.
        # status: phoenix6.StatusCode = phoenix6.StatusCode.STATUS_CODE_NOT_INITIALIZED
        # for _ in range(0, 5):
        #     # Apply configuration and check its status.
        #     status = self.kraken.configurator.apply(configuration)
        #     if status.is_ok():
        #         break
        # if not status.is_ok():
        #     print(f"Could not apply configs, error code: {status.name}")

        # ----------------------------------------------------------------------
        # Create several control request objects that we will send to the
        # # Kraken to describe what we want it to do.

        # # Either brake or coast, depending on motor configuration; we chose brake above.
        # self.brake_request = phoenix6.controls.NeutralOut()

        # # Position request starts at position 0, but can be modified later.
        # self.position_request = phoenix6.controls.PositionVoltage(0).with_slot(0)

        # # A motion magic (MM) position request. MM smooths the acceleration.
        # self.mm_pos_request = phoenix6.controls.MotionMagicVoltage(0).with_slot(1)

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        # if self.print_timer.advanceIfElapsed(0.2):
        #     wpilib.SmartDashboard.putString(
        #         "DB/String 0",
        #         "rotations: {:5.1f}".format(self.kraken.get_position().value),
        #     )
        pass

    def disabledExit(self):
        pass

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # Get control values so we can manipulate them if desired.
        # left = self.xbox.getLeftY()
        # right = self.xbox.getRightY()

        # # Command motor operation.
        # # self.motor2.set(left)
        # # self.motor4.set(right)

        # # Kraken, either brake or move to a position
        # desired_rotations = right * 5
        # if abs(desired_rotations) <= 0.1:
        #     desired_rotations = 0

        # if self.xbox.getRightBumper():
        #     self.kraken.set_control(
        #         self.position_request.with_position(desired_rotations)
        #     )
        # elif self.xbox.getLeftBumper():
        #     self.kraken.set_control(
        #         self.mm_pos_request.with_position(desired_rotations)
        #     )
        # else:
        #     self.kraken.set_control(self.brake_request)

        # if self.print_timer.advanceIfElapsed(0.2):
        #     wpilib.SmartDashboard.putString(
        #         "DB/String 0",
        #         "rotations: {:5.1f}".format(self.kraken.get_position().value),
            # )
        a = self.xbox.getAButton()
        
        if a:
            self.motor2.set(1.0)
        else:
            self.motor2.set(0.0)

    def teleopExit(self):
        pass


if __name__ == "__main__":
    wpilib.run(Myrobot)
