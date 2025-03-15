import commands2

from subsystems.drive_subsystem import DriveSubsystem


class ResetGyroCommand(
    commands2.Command
):  # Class type command from the library command2
    def __init__(self, drive_subsystem: DriveSubsystem):
        super().__init__()
        self.drive_subsystem = drive_subsystem

        self.addRequirements(drive_subsystem)  # Requires this subsystem

    def execute(self):
        print("reset gyro")
        self.drive_subsystem.gyro.set_yaw(0.0)

    def isFinished(self) -> bool:
        return True
