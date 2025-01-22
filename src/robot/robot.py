import commands2
import wpilib 
import wpimath 

import robot_container
from subsystems import drive_subsystem
from wpimath import applyDeadband

# Robot Class
class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = robot_container.RobotContainer()
        self.drive_subsystem = drive_subsystem.DriveSubsystem()

    def disabledPeriodic(self):
        pass

    def teleopPeriodic(self):
        self.desired_Ydeadband = applyDeadband(self.container.drive_controller.getLeftY(), 0.5)
        self.desired_Xdeadband = applyDeadband(self.container.drive_controller.getLeftX(), 0.5) 
        self.a = self.container.drive_controller.getAButton()

        desired_pos = int(self.drive_subsystem.get_cancoder())

        wpilib.SmartDashboard.putString('Left Y', 'at: {:5.1f}'.format(self.desired_Ydeadband))
        wpilib.SmartDashboard.putString('Left X', 'at: {:5.1f}'.format(self.desired_Xdeadband))

        if self.a:
            self.drive_subsystem.turn_motor.set_control(self.drive_subsystem.mm_pos_request.with_position(desired_pos))
            print("Going to 0 of CAN")
