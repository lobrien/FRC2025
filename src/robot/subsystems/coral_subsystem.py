import commands2
import wpilib
import rev
from constants.coralconsts import CoralConsts
from constants.new_types import inches


class CoralSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()  # Call the Subsystem class's (the "super" part) init.

        # Motor object 
        self.coral_motor = rev.SparkMax(CoralConsts.CORAL_MOTOR, rev.SparkMax.MotorType.kBrushless)

        # Make a configuration object
        config = rev.SparkMaxConfig()

        # Basic parts of the configuration.
        config.IdleMode(rev.SparkMax.IdleMode.kBrake)
        config.inverted(False) #TODO: update?

        # Apply it to the motor.
        self.coral_motor.configure(config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    def intake(self):
        """
        This makes the motor move the wheels, like *shoop*
        """
        self.coral_motor.set(1.0) # TODO: update?

    def outtake(self):
        """
        This makes the motor move the wheels the other way, like *woosh*
        """
        self.coral_motor.set(-1.0) # TODO: update?

    def stop(self):
        """
        Stopping the motor from making coral go *smack!* agaist the back board
        """
        self.coral_motor.set(0.0)

    def is_coral_loaded(self) -> bool:
        """
        Checking the motor to see if it reaches the stop current, if does return true
        """
        motor_current = self.coral_motor.getOutputCurrent()

        if motor_current >= CoralConsts.CORAL_STOP_CURRENT: # If the current isn't exactly the constants then it would never stop, this is so that even if the current checked gos over the constants then it would still stop
            return True
        else:
            return False
        