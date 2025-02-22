import commands2
import rev
from constants.algaeconsts import AlgaeConsts

class AlgaeSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()  # Call the Subsystem class's (the "super" part) init.

        # Motor object 
        self.algae_motor = rev.SparkMax(AlgaeConsts.ALGAE_MOTOR, rev.SparkMax.MotorType.kBrushless)

        # Make a configuration object
        config = rev.SparkMaxConfig()

        # Basic parts of the configuration.
        config.IdleMode(rev.SparkMax.IdleMode.kBrake)
        config.inverted(False) #TODO: update?

        # Apply it to the motor.
        self.algae_motor.configure(config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    def intake(self):
        """
        This makes the motor move the wheels, like *shoop*
        """
        self.algae_motor.set(1.0) # TODO: update?

    def outtake(self):
        """
        This makes the motor move the wheels the other way, like *woosh*
        """
        self.algae_motor.set(-1.0) # TODO: update?

    def stop(self):
        """
        Stoping the motor, like NO MORE MOVEMENT ALLOWED!
        """

        self.algae_motor.set(0.0)
    
    def is_algae_loaded(self) -> bool:
        """
        Checking the motor to see if it reaches the stop current, if does return true
        """
        motor_current = self.algae_motor.getOutputCurrent()

        if motor_current >= AlgaeConsts.ALGAE_STOP_CURRENT: # If the current isn't exactly the constants then it would never stop, this is so that even if the current checked gos over the constants then it would still stop
            return True
        else:
            return False