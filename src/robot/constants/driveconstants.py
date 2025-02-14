from wpimath.geometry import Translation2d
from dataclasses import dataclass, field

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
    # CANCoder (magnet) offsets in rotations
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
    # TODO: replace with correct ID.
    PIGEON_ID = 13

    # Drivetrain geometry, gearing, etc.
    # TODO: Consistent units! We agreed inches and degrees!
    TRACK_HALF_WIDTH = 0.27       # meters (21.25 in track width)   
    # TODO: Consistent units! We agreed inches and degrees!
    WHEELBASE_HALF_LENGTH = 0.27 # meters (21.25 in wheelbase)
    TURN_GEAR_RATIO = 468.0/35.0     # Kraken 
    DRIVE_GEAR_RATIO = 9 #temp
    WHEEL_DIA = 4  # 4" diameter
    WHEEL_RADIUS = WHEEL_DIA / 2

    # Free speed from https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/generated/TunerConstants.java
    # TODO: This needs to be changed to our measured velocity at max voltage (place holder value is phoenix6 @ 12V according to above link)
    # TODO: Consistent units! We agreed inches and degrees!
    FREE_SPEED = 3.76 # m/s

    # TODO: Base this on the free speed.
    MAX_SPEED_INCHES_PER_SECOND = 145.7 / 4 # inches per second

    # TODO: add a maximum rotational speed in degrees/second

    # TODO: Delete this code? Do not leave commented out code in the codebase.
    # # PID controller constants (gains)
    # # Proportional constant only at the moment, all others assumed zero.
    # PIDX_KP = 1       # X dimension PID controller's proportional constant
    # PIDY_KP = 1       # Y dimension PID controller's proportional constant
    # PID_ROT_KP = 1    # Rotation controller's proportional constant.
    # # Horizontal (x or y) maxima and tolerances
    # HORIZ_MAX_V = 1   # Maximum velocity in meters/second
    # HORIZ_MAX_A = 2  # Maximum acceleration in meters/second/second
    # HORIZ_POS_TOL = 0.1 # Position tolerance in meters (within this distance is "close enough")
    # HORIZ_VEL_TOL = 0.01 # Velocity tolerance in meters/second
    # # Rotational maxima and tolerances
    # # TODO: Confirm these are radians, radians/sec, radians/sec^2?
    # ROT_MAX_V = 0.1   # Rotational maximum velocity
    # ROT_MAX_A = 0.1   # Rotational maximum acceleration
    # ROT_POS_TOL = 0.1 # Rotational position tolerance
    # ROT_VEL_TOL = 0.1 # Rotational velocity tolerance
    # SWERVE_MODULE_TURN_PID_KP = 0.3 # TODO: This is just what it was in 2024 CrescendoSwerveModule.py line 81
    # SWERVE_MODULE_TURN_PID_KI = 0.0
    # SWERVE_MODULE_TURN_PID_KD = 0.0