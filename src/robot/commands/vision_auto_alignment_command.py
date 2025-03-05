import commands2
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.drive_subsystem import DriveSubsystem

class VisionAutoAlign(commands2.Command):
        def __init__(self, vision_subsystem:VisionSubsystem, drive:DriveSubsystem):
            super().__init__()
            self.drive_subsystem = drive

            self.addRequirements(vision_subsystem)
            self.addRequirements(drive)

        def execute(self):
            
            desired_x = 7.5
            if self.is_botpose_valid(self.botpose):
                self.bot_x = self.botpose[0]
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