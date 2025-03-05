import math
import ntcore
import commands2

import time # Temporary for diagnostics

class VisionSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        # self.desired_x_for_autonomous_driving = desired_auto_x
        # self.reef _x = reef _x

        self.networktables = ntcore.NetworkTableInstance.getDefault()
        self.limelight_table = self.networktables.getTable("limelight")
        self.botpose_subscription = self.limelight_table.getDoubleArrayTopic("botpose").subscribe([])
        self.botpose = [-1, -1, -1, -1, -1, -1]

    def checkBotpose(self):
        botpose = self.botpose_subscription.get()
        # Only modify botpose if: it exists, it's the correct datastructure, and it's not all zeros
        if botpose is not None: # and len(botpose > 3) and botpose[0] + botpose[1] + int(botpose[2] != 0):  <- the code behind gives an error saying you cannot combine ints and lists
            self.botpose = botpose
        return self.botpose
    
    def calculate_desired_x_distance(self, desired_x, bot_x):
        desired_pos = desired_x - bot_x
        return desired_pos
            

    def calculate_desired_direction(self, desired_angle, current_angle):
        """
        this function calculates the direction to travel for the robots yaw by saying if the value is greater than 180 it returns a negative number in degrees needed to travel
        if it is less it returns a positive amount of degrees to travel to get to the desired position.
        """
        desired_direction = desired_angle - current_angle
        if desired_direction > 180:
            desired_direction -= 360
        if desired_direction < -180:
            desired_direction += 360
        return desired_direction
    
    def distance_to_reef (self, bot_x, bot_y, reef_x, reef_y):
        '''
        distance to reef  calculates the distance from the robots pos to the reef  in meters. it uses the distance formula subtracting the desired x,y (the reef )
        by our current x, y and square roots the awnser to get our distance to be used in our shooter angle calculations. returns the distance from the robot to the reef 
        '''
        distance = math.sqrt(pow(2, reef_x - bot_x)) + (pow(2, reef_y - bot_y))
        return distance
    def calculate_desired_angle(self, distance_to_reef_y, distance_to_wall):
        """
        This function calculates the desired angle for the robot's orientation at different positions. It's measured by getting the distance to reef  on the y axis and the distance to the wall using the x axis
        and dividing them and uses the arctan function to calculate the angle needed to for the robot to rotate to into the reef  angle. This function returns the robots needed orientation in degrees.
        distance to reef Y is the adjacent angle
        distance to wall is the opposite angle
        """
        desired_angle_rad = math.atan2(distance_to_reef_y, distance_to_wall) 
        desired_angle = self.radians_to_degrees(desired_angle_rad)
        if desired_angle < 0:
            desired_angle += 360
        return desired_angle
    
    def radians_to_degrees(self, radians):
        '''
        calculates radians into degrees
        '''
        return radians * (180/math.pi)
    
    def get_rotation_autonomous_periodic_for_reef_shot(self, botpose, current_yaw):
        x = botpose[0]
        # desired_x = self.desired_x_for_autonomous_driving
        y = botpose[1]
       

        reef_y = 1.44 # 
        # distance_to_wall = (self.reef _x - x) #Ajd
        # distance_to_reef _y = (reef _y - y) #Opp
        # # reef _distance = self.distance_to_reef (x, y, reef _x, reef _y) #Hy

        # desired_bot_angle = self.calculate_desired_angle(distance_to_reef _y, distance_to_wall)


        # direction_to_travel = self.calculate_desired_direction(desired_bot_angle, current_yaw)
        # direction_to_travel = self.radians_to_degrees(direction_to_travel_rad)
        # x_distance_to_travel = (desired_x - x)
        # x_kp = 0.007
        # x_max_speed = 0.2
        # # x_speed = x_kp * x_distance_to_travel

        # yaw_kp = 0.007
        # max_rot_value = 0.3
        # rot = yaw_kp * direction_to_travel
        # this acts like the p value in a pid loop for the rotation action

        # if x_speed > x_max_speed:
        #     x_speed = x_max_speed
        # elif x_speed < -x_max_speed:
        #     x_speed = -x_max_speed

        # if rot > max_rot_value:
        #     rot = max_rot_value
        # elif rot < -max_rot_value: 
        #     rot = -max_rot_value
        #     # this sets makes sure that the rot value does not pass the maximum we give

        # return rot, direction_to_travel

    
