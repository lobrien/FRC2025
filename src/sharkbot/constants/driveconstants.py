from wpimath.geometry import Translation2d
from dataclasses import dataclass, field

@dataclass(frozen=True)
class DriveConstants:
    """ Drivetrain related constants"""
    #CAN bus IDs
    CAN_FR = 11 # Front right
    CAN_FL = 8 # Front left 
    CAN_BL = 5 # Back left
    CAN_BR = 2 # Back right

    # Kraken IDs
    DRIVE_FR = 12 # Front right
    DRIVE_FL = 9 # Front left 
    DRIVE_BL = 6 # Back left
    DRIVE_BR = 3 # Back right

    TURN_FR = 10 # Front right
    TURN_FL = 7 # Front left 
    TURN_BL = 4 # Back left
    TURN_BR = 1 # Back right

    #CAN offsets

    # Drivetrain geometry, gearing, etc.
    TRACK_HALF_WIDTH = 0.18       # meters (36 cm track width)
    WHEELBASE_HALF_LENGTH = 0.225 # meters (45 cm wheelbase)
    GEAR_RATIO = 12.75      # Toughbox micro
    WHEEL_DIA = 4 * 0.0254  # 4" diameter in meters

    # PID controller constants (gains)
    # Proportional constant only at the moment, all others assumed zero.
    PIDX_KP = 1       # X dimension PID controller's proportional constant
    PIDY_KP = 1       # Y dimension PID controller's proportional constant
    PID_ROT_KP = 1    # Rotation controller's proportional constant.
    # Horizontal (x or y) maxima and tolerances
    HORIZ_MAX_V = 1   # Maximum velocity in meters/second
    HORIZ_MAX_A = 2  # Maximum acceleration in meters/second/second
    HORIZ_POS_TOL = 0.1 # Position tolerance in meters (within this distance is "close enough")
    HORIZ_VEL_TOL = 0.01 # Velocity tolerance in meters/second
    # Rotational maxima and tolerances
    # TODO: Confirm these are radians, radians/sec, radians/sec^2?
    ROT_MAX_V = 0.1   # Rotational maximum velocity
    ROT_MAX_A = 0.1   # Rotational maximum acceleration
    ROT_POS_TOL = 0.1 # Rotational position tolerance
    ROT_VEL_TOL = 0.1 # Rotational velocity tolerance