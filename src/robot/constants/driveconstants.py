from dataclasses import dataclass
from wpimath.units import metersToInches

from constants.new_types import (
    inches_per_second,
    inches,
    degrees_per_second,
    degrees,
    inches_per_second_squared,
    degrees_per_second_squared,
)


@dataclass(frozen=True)
class DriveConstants:
    """Drivetrain related constants"""

    # CANCoder CAN bus IDs
    CAN_FR = 11  # Front right
    CAN_FL = 8  # Front left
    CAN_BL = 5  # Back left
    CAN_BR = 2  # Back right

    # CANCoder (magnet) offsets in rotations that we got from the CANCoder using Phoenix Tuner X
    BR_OFFSET = -0.206
    BL_OFFSET = -0.306
    FR_OFFSET = -0.450
    FL_OFFSET = 0.441

    # Kraken IDs
    DRIVE_FR = 12  # Front right
    DRIVE_FL = 9  # Front left
    DRIVE_BL = 6  # Back left
    DRIVE_BR = 3  # Back right

    TURN_FR = 10  # Front right
    TURN_FL = 7  # Front left
    TURN_BL = 4  # Back left
    TURN_BR = 1  # Back right

    # Pigeon2 gyro CAN bus ID
    PIGEON_ID = 13

    # Drivetrain geometry, gearing, etc.
    TRACK_HALF_WIDTH: inches = metersToInches(0.27)  # meters (21.25 in track width)
    WHEELBASE_HALF_LENGTH: inches = metersToInches(0.27)  # meters (21.25 in wheelbase)
    TURN_GEAR_RATIO = 468.0 / 35.0  # Kraken
    DRIVE_GEAR_RATIO = 6.2 #606.1 / 96.5
    WHEEL_DIA: inches = 4  # 4" diameter
    WHEEL_RADIUS: inches = WHEEL_DIA / 2

    FREE_SPEED = 3.76  # max velocity collected from giving the motors max input. Not currently used anywhere
    # TODO: Change FREE_SPEED above to the measured motor RPM, and then use that in
    # an explicit calculation of MAX_SPEED_INCHES_PER_SECOND that uses the gear ratio and wheel diameter.
    # Then if either of those constants change, the max speed will change appropriately.
    # It also makes the value's origin clearer.

    # Based on measurement of motor RPM with the robot up on blocks, adjusted
    # for gear ratio and wheel diameter.
    # The "/ SLOWED_FACTOR" is to slow things down for testing.
    MAX_SPEED_INCHES_PER_SECOND: inches_per_second = (
        145.7 
    )  # inches per second
    MAX_DEGREES_PER_SECOND: degrees_per_second = (
        72.85
    )  # degrees per second
    
    # PID controller constants (gains)
    # Proportional constant only at the moment, all others assumed zero.
    # For X and Y, 1 meter error results in a motor command of 1.0 (full voltage).
    # Converted to 1/0.0254 = 39 inches => full voltage.
    # For rotation, 90 degrees error => full voltage.
    PIDX_KP: float = 1.0 * 0.9592 # X dimension PID controller's proportional constant
    PIDY_KP: float = 1.0 * 0.9592   # Y dimension PID controller's proportional constant
    PID_ROT_KP: float = 1.0 / 90.0  # Rotation controller's proportional constant.

    # Horizontal (x or y) maxima and tolerances
    HORIZ_MAX_V: inches_per_second = 39.0  # Maximum velocity in inches/second
    HORIZ_MAX_A: inches_per_second_squared = (
        78.0 * 1.5 # Maximum acceleration in inches/second/second
    )
    HORIZ_POS_TOL: inches = (
        4.0  # Position tolerance in inches (within this distance is "close enough")
    )
    HORIZ_VEL_TOL: inches_per_second = 0.4  # Velocity tolerance in inches/second

    # Rotational maxima and tolerances
    ROT_MAX_V: degrees_per_second = (
        40.0  # Rotational maximum velocity in degrees/second
    )
    ROT_MAX_A: degrees_per_second_squared = (
        20.0  # Rotational maximum acceleration in degrees/second/second
    )
    ROT_POS_TOL: degrees = 5.0  # Rotational position tolerance in degrees
    ROT_VEL_TOL: degrees_per_second = (
        1.0  # Rotational velocity tolerance in degrees/second
    )
