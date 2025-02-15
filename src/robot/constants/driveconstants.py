from dataclasses import dataclass
from wpimath.units import metersToInches

@dataclass(frozen=True)
class DriveConstants:
    """ Drivetrain related constants"""
    # CANCoder CAN bus IDs
    CAN_FR = 11 # Front right        
    CAN_FL = 8 # Front left         
    CAN_BL = 5 # Back left           
    CAN_BR = 2 # Back right         

    # TODO: Consistent units! We agreed inches and degrees! OR write clear comment explaining why this should be
    # in rotations
    # CANCoder (magnet) offsets in rotations that we got from the CANCoder using Phoenix Tuner X
    BR_OFFSET = -0.206
    BL_OFFSET = -0.306
    FR_OFFSET = -0.450
    FL_OFFSET = 0.441

    # Kraken IDs
    DRIVE_FR = 12 # Front right
    DRIVE_FL = 9 # Front left 
    DRIVE_BL = 6 # Back left
    DRIVE_BR = 3 # Back right

    TURN_FR = 10 # Front right
    TURN_FL = 7 # Front left 
    TURN_BL = 4 # Back left
    TURN_BR = 1 # Back right

    # Pigeon2 gyro CAN bus ID
    PIGEON_ID = 13

    # Drivetrain geometry, gearing, etc.  
    TRACK_HALF_WIDTH = metersToInches(0.27)       # meters (21.25 in track width)   
    WHEELBASE_HALF_LENGTH = metersToInches(0.27)  # meters (21.25 in wheelbase)
    TURN_GEAR_RATIO = 468.0/35.0     # Kraken 
    DRIVE_GEAR_RATIO = 9 #temp
    WHEEL_DIA = 4  # 4" diameter
    WHEEL_RADIUS = WHEEL_DIA / 2

    # TODO: Consistent units! We agreed inches and degrees! 
    # 
    FREE_SPEED = 3.76 # max veloctity colected from giving the motors max input. Not currently used any where

    SLOWED_FACTOR = 4

    # TODO: Change FREE_SPEED above to the measured motor RPM, and then use that in
    # an explicit calculation of MAX_SPEED_INCHES_PER_SECOND that uses the gear ratio and wheel diameter.
    # Then if either of those constants change, the max speed will change appropriately.
    # It also makes the value's origin clearer.

    # Based on measurement of motor RPM with the robot up on blocks, adjusted
    # for gear ratio and wheel diameter.
    # The "/ SLOWED_FACTOR" is to slow things down for testing.
    MAX_SPEED_INCHES_PER_SECOND = 145.7 / SLOWED_FACTOR # inches per second
    MAX_DEGREES_PER_SECOND = 72.85 / SLOWED_FACTOR # degrees per second
