# Origin is bottom blue alliance
from dataclasses import Field, dataclass

from math import cos, sin
from wpilib import DriverStation
from wpimath.units import degreesToRadians

from util.field_calculations import center_relative_offset, reef_relative_offset
from constants.new_types import inches



# The field is 57' 5 1/2" long and 26' 5 1/2" wide (Ref. Reefscape Field Reference TODO: NEED URL )
@dataclass(frozen=True)
class FieldConstants:
    """Field related constants"""

    FIELD_LENGTH: inches = 689.5  # 57' 5 1/2"
    FIELD_WIDTH: inches = 317

    FIELD_MID_LENGTH: inches = FIELD_LENGTH / 2
    FIELD_MID_WIDTH: inches = FIELD_WIDTH / 2

    # Constants for nominal starting positions: both offsets are |DISTANCE| from the middle of the field

    # You MUST only call these via FieldConstants.offset() to ensure that they are always adjusted
    # For instance, MID_START_X, MID_START_Y = FieldConstants.offset(12, 0)

    _MID_TO_ROBOT_X : inches = 24
    MID_START_X, MID_START_Y = center_relative_offset(_MID_TO_ROBOT_X, 0)
    OFFSET_START_X, OFFSET_START_Y = center_relative_offset(_MID_TO_ROBOT_X, 6 * 12)  # Position robot so that it's field position is (START_AREA_EDGE, 6') from middle(either direction)

    REEF_CENTER_X, REEF_CENTER_Y = center_relative_offset(177, 0)  # 14'9" away from the middle
    REEF_INNER_WIDTH = 65.5 # face-to-face, not corner-to-corner
