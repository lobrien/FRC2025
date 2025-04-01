import commands2
import wpilib
import rev
from constants.coralconsts import CoralConsts
from constants.new_types import inches
from wpilib import SmartDashboard
from util.current_threshold import CurrentThreshold


class CoralSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()  # Call the Subsystem class's (the "super" part) init.
        self.current_threshold = CurrentThreshold("Coral Intake", CoralConsts.CORAL_STOP_CURRENT)

        # Motor object 
        self.coral_motor = rev.SparkMax(CoralConsts.CORAL_MOTOR, rev.SparkMax.MotorType.kBrushless)

        # Make a configuration object
        config = rev.SparkMaxConfig()

        # Basic parts of the configuration.
        config.IdleMode(rev.SparkMax.IdleMode.kBrake)
        config.inverted(False) #SparkMax lock (breaks)

        # Apply it to the motor.
        self.coral_motor.configure(config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    def intake(self):
        """
        This makes the motor move the wheels, like *shoop*
        """
        self.coral_motor.set(0.17) #positive imput spinning wheel inward at 17%

    def outtake(self):
        """
        This makes the motor move the wheels the other way, like *woosh*
        """
        self.coral_motor.set(-0.25) #negative imput spinning wheel outward at 25%

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
        
    def periodic(self):
        motor_current = self.coral_motor.getOutputCurrent()
        
        self.current_threshold.is_exceeded(motor_current)
    