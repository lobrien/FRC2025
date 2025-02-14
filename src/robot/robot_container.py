import commands2
import wpilib
from commands2.button import CommandXboxController
from wpimath import applyDeadband
from commands2 import RepeatCommand
from constants.autoconsts import AutoConsts
from wpilib import SmartDashboard, SendableChooser
from wpilib.shuffleboard import Shuffleboard


from commands.print_something_command import PrintSomethingCommand
from constants.operatorinterfaceconstants import OperatorInterfaceConstants
from subsystems.drive_subsystem import DriveSubsystem
from commands.drive_forward_command import DriveForwardCommand
from commands.drive_with_joystick_command import DriveWithJoystickCommand
from commands.turn_to_angle_command import TurnToAngleCommand
from commands.reset_gyro_command import ResetGyroCommand

# The `RobotContainer` class is where the robot's structure and behavior are defined.
# It is the glue that holds the robot together.
# Hardware-wise:
# * The `RobotContainer` has group of Subsystems. When you add or delete a complete
#   subsystem, you will need to update the `RobotContainer`.
# * The `RobotContainer` has one or more input controllers (e.g. Xbox controllers, joysticks).
#
# Command-wise:
# * The `RobotContainer` instantiates the Commands and CommandGroups that make
#   up the robot's behavior. These commands are created and wired together here
#  in the `RobotContainer`. If you ever want to review the robot's behavior, the
#  `RobotContainer` is the proper place to look. Even the autonomous and default commands
#   should be instantiated here.
#* The `RobotContainer` maps between the input devices and the Commands that operate on the Subsystems.
class RobotContainer:
    def __init__(self):
        self.drive_subsystem = DriveSubsystem() #Update the array when ready
        self.controller = CommandXboxController(OperatorInterfaceConstants.DRIVER_CONTROLLER_PORT)

        self.teleop_command = DriveWithJoystickCommand(self.drive_subsystem, self.get_drive_value_from_joystick)

        self.controller.a().onTrue(PrintSomethingCommand("WHEA A Button Pressed"))
        self.controller.b().whileTrue(TurnToAngleCommand(self.drive_subsystem, lambda: True)) #for quick test 
        
        self.drive_subsystem.setDefaultCommand(self.teleop_command) #Set the teleop command as the default for drive subsystem
    
        self.controller.leftBumper().and_(self.controller.rightBumper()).whileTrue(ResetGyroCommand(self.drive_subsystem))


    def get_auto_command(self) -> commands2.Command:
        """
        We moved the code that makes the auto chooser and making the default option for it from init to the get auto command function because
        when we did it normally it gave us an error saying there is no auto chooser so we just made one here. is there a way to fix this?

        TODO: In the future, use TODO: to mark a task or question like ^. We can search for all TODO:s in the codebase to find them.
        """

         #Auto chooser
        self.auto_chooser = SendableChooser()

        self.auto_chooser.setDefaultOption("Forward", AutoConsts.FORWARD)  #Autos.side_step(self.drive)
        
        #Add options

        #Add chooser to tab
        SmartDashboard.putData("Auto Commmand Selector", self.auto_chooser)

        auto_reader = self.auto_chooser.getSelected()

        if auto_reader == AutoConsts.FORWARD:
           return DriveForwardCommand(self.drive_subsystem, duration=5)

    


    def get_drive_value_from_joystick(self) -> tuple[float, float, float]:
        """
        Gets joystick values and scales them for improved operator control.
        Returns:
            Tuple of percentage values in the three joystick axes, leftX, leftY, rightX.
        """
        x_percent = applyDeadband(value=self.controller.getLeftX(), deadband=0.1) #Apply deadband to the values above
        y_percent = applyDeadband(value=self.controller.getLeftY(), deadband=0.1)
        rot_percent = applyDeadband(value=self.controller.getRightX(), deadband=0.1)

        x_percent = self.joystickscaling(x_percent)
        y_percent = self.joystickscaling(y_percent)
        rot_percent = self.joystickscaling(rot_percent)

        return x_percent, y_percent, rot_percent  #Gives us these variables when we call this function
    
    def joystickscaling(self, input): #this function helps bring an exponential curve in the joystick value and near the zero value it uses less value and is more flat
        a = 1
        output = a * input * input * input + (1 - a) * input
        return output




 