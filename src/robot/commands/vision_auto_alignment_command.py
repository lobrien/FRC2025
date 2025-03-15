import commands2
from wpimath.geometry import Pose2d
from wpimath.units import metersToInches, inchesToMeters, degreesToRadians
import logging

from constants.new_types import inches, degrees, inches_per_second
from subsystems.drive_subsystem import DriveSubsystem

logger = logging.getLogger(__name__)

class VisionAutoAlign(commands2.Command):
        def __init__(self, 
                     drive:DriveSubsystem,
                     desired_field_relative_position_inches : tuple[inches, inches],
                     desired_field_relative_angle_degrees : degrees):
            """
            VisionAutoAlign Command.

            Takes a field-relative position and angle, and drives the robot to that position and angle.
            """
            super().__init__()
            
            # Pose2d is defined with meters and radians, so convert inches and degrees appropriately:
            desired_x_meters = inchesToMeters(desired_field_relative_position_inches[0])
            desired_y_meters = inchesToMeters(desired_field_relative_position_inches[1])
            desired_yaw_radians = degreesToRadians(desired_field_relative_angle_degrees)
            self.desired_pose = Pose2d(desired_x_meters, desired_y_meters, desired_yaw_radians)

            self.drive_subsystem = drive

            self.addRequirements(drive)

        def isFinished(self) -> bool:
            # If there is no botpose, we cannot align
            bot_pose = self.drive_subsystem.get_estimated_pose()
            if bot_pose is None:
                logger.error("No botpose available, cannot align")
                return True
            # TODO: These indices need to be confirmed and put into constants (e.g., "BOTPOSE_X_INDEX")
            robot_relative_pose = self.desired_pose.relativeTo(Pose2d(bot_pose[0], bot_pose[1], bot_pose[5]))
            # TODO: Convert these tolerances to constants
            if VisionAutoAlign._close_enough(robot_relative_pose, inches(1), degrees(5)):
                return True
            else:
                return False

        @staticmethod
        def _close_enough(pose_relative_to_desired: Pose2d, transform_tolerance : inches, rotation_tolerance : degrees) -> bool:
            pose_x_inches = metersToInches(pose_relative_to_desired.translation().x)
            pose_y_inches = metersToInches(pose_relative_to_desired.translation().y)
            pose_yaw_degrees = degrees(pose_relative_to_desired.rotation().degrees())

            distance = (pose_x_inches**2 + pose_y_inches**2)**0.5
            rotation = abs(pose_yaw_degrees)
            if distance < transform_tolerance and rotation < rotation_tolerance:
                return True
            else:
                return False

        def execute(self):

                ### REWORK THIS CODE FROM HERE ###
                # Step 1: Get the botpose from the drive
                bot_pose = self.drive_subsystem.get_estimated_pose()
                if bot_pose is None:
                    logger.error("No botpose available, cannot align")
                else:
                    # Step 2: Calculate distance and rotation to desired position
                    robot_relative_pose = self.desired_pose.relativeTo(Pose2d(bot_pose[0], bot_pose[1], bot_pose[5]))
                    # Step 3: Calculate speeds for next 20ms
                    raise NotImplementedError("Calculate speeds")
                    x_speed : inches_per_second = 0
                    y_speed : inches_per_second = 0
                    rot_speed : degrees_per_second = 0
                    # Step 4: Send power to drive subsystem
                    self.drive_subsystem.drive(x_speed, y_speed, rot_speed)


                    #### CURRENT CODE TO REWORK INTO STEPS above
                    if self.is_botpose_valid(self.botpose):
                        bot_x = self.botpose[0]
                    robot_x = self.botpose[0]
                    robot_y = self.botpose[1]
                    robot_yaw = self.botpose[5]

                    desired_bot_angle = 0
                    desired_x_pos = desired_x - robot_x

                    desired_direction = desired_bot_angle - robot_yaw
                    if desired_direction > 180:
                        desired_direction -= 360
                    if desired_direction < -180:
                        desired_direction += 360

                    x_kp = 0.3
                    x_max_speed = 0.5
                    self.x_speed = x_kp * desired_x_pos

                    # this acts like the p value in a pid loop for the rotation action

                    if self.x_speed > x_max_speed:
                        self.x_speed = x_max_speed
                    elif self.x_speed < -x_max_speed:
                        self.x_speed = -x_max_speed
                    #     # this sets makes sure that the rot value does not pass the maximum we give

                    self.x_distance = desired_x_pos

                    if robot_x > -0.1 and robot_x < 0.1:
                        self.x_speed = 0.0
                    elif desired_x_pos > -0.5:
                        self.x_speed = -self.x_speed
                    elif abs(desired_x_pos) < 3:
                        self.x_speed = 0.0
                    else:
                        self.x_speed = self.x_speed + 0.25

                    print(f"speed is{self.x_speed}")
                    print(f"distance is{desired_x_pos}")


                    desired_y = 1.45
                    desired_y_pos = desired_y - robot_y

                    y_kp = 0.3
                    y_max_speed = 0.4
                    self.y_speed = y_kp * desired_y_pos

                    # this acts like the p value in a pid loop for the rotation action

                    if self.y_speed > y_max_speed:
                        self.y_speed = y_max_speed
                    elif self.y_speed < -y_max_speed:
                        self.y_speed = -y_max_speed

                    if robot_y > -0.1 and robot_y < 0.1:
                        self.y_speed = 0.0
                    elif desired_y_pos > -0.5:
                        self.y_speed = -self.y_speed
                    elif abs(desired_y_pos) < 3:
                        self.y_speed = 0.0
                    else:
                        self.y_speed = self.y_speed


                    # yaw_kp = 0.7
                    # max_rot_value = 0.3
                    # rot = yaw_kp * desired_direction

                    # if rot > max_rot_value:
                    #     rot = max_rot_value
                    # elif rot < -max_rot_value:
                    #     rot = -max_rot_value

                    # if robot_yaw > -0.1 and robot_yaw < 0.1:
                    #     self.rot = 0.0
                    # elif desired_bot_angle > -0.5:
                    #     self.rot = -self.rot
                    # elif abs(desired_bot_angle) < 1:
                    #     self.rot = 0.0
                    # else:
                    #     self.rot = self.rot

                    print(self.rot)

                    self.drive_subsystem.drive(self.x_speed, self.y_speed, 0.0) #self.rot)