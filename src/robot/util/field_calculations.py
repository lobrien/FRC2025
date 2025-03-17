from math import cos, sin
from wpilib import DriverStation
from wpimath.geometry import Pose2d
from wpimath.units import degreesToRadians

from constants.fieldconstants import FieldConstants
from constants.new_types import inches

# TODO IMPORTANT: We must be 100% confident in these calculations.

def _alliance_sign() -> int:
    """Returns the sign of the alliance"""
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        return 1
    elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
        return -1
    else:
        raise ValueError("Invalid alliance")

def center_relative_offset(x: inches, y: inches) -> tuple[inches, inches]:
    """Returns the offset of a field constant"""
    from constants.fieldconstants import FieldConstants  # Runtime import to avoid circular import

    # Calculate the offsets to the middle of the field
    field_x = (FieldConstants.FIELD_MID_LENGTH + x) * _alliance_sign()
    field_y = (FieldConstants.FIELD_MID_WIDTH + y) * _alliance_sign()

    return (field_x, field_y)

class Reef:

    @staticmethod
    def reef_relative_offset(x: inches, y: inches) -> tuple(inches, inches):
        """Returns the offset of a field constant"""
        from constants.fieldconstants import FieldConstants  # Runtime import to avoid circular import

        # Since Blue is left of midfield, this is +- x axis
        x_sign = 1 if DriverStation.getAlliance() == DriverStation.Alliance.kRed else -1
        # Since Red is below the middle, this is +- y axis
        y_sign = -1 if DriverStation.getAlliance() == DriverStation.Alliance.kRed else 1

        # Calculate the offsets to the middle of the field
        reef_x = (FieldConstants.FieldConstants.REEF_CENTER_X + x) * x_sign
        reef_y = (FieldConstants.FieldConstants.REEF_CENTER_Y + y) * y_sign

        return (reef_x, reef_y)

def get_reef_face_pose2d(oclock : int, facing_center : bool = False, standoff : inches = 0) -> Pose2d

    # Constants for the reef
    # To *face* the reef, rotate by 180 degrees
    

    REEF_APOTHEM : inches = FieldConstants.REEF_INNER_WIDTH / 2

    # Reef faces. Clock is viewed from above with 12 o'clock at top (+Y). Numbers are "o'clock" position, rounded down to hour (i.e., REEF_FACE_1 is the face at 1:30)
    # Hex faces are 60 degrees apart, so we can use trigonometry to calculate the coordinates of each face
    # The face at 1:30 is at 30 degrees, the face at 3 is at 90 degrees, etc.
    _REEF_FACE_1_X = FieldConstants.REEF_CENTER_X + REEF_APOTHEM * cos(degreesToRadians(30))
    _REEF_FACE_1_Y = FieldConstants.REEF_CENTER_Y + REEF_APOTHEM * sin(degreesToRadians(30))

    _REEF_FACE_3_X = FieldConstants.REEF_CENTER_X + REEF_APOTHEM * cos(degreesToRadians(90))
    _REEF_FACE_3_Y = FieldConstants.REEF_CENTER_Y + REEF_APOTHEM * sin(degreesToRadians(90))

    _REEF_FACE_5_X = FieldConstants.REEF_CENTER_X + REEF_APOTHEM * cos(degreesToRadians(150))
    _REEF_FACE_5_Y = FieldConstants.REEF_CENTER_Y + REEF_APOTHEM * sin(degreesToRadians(150))

    _REEF_FACE_7_X = FieldConstants.REEF_CENTER_X + REEF_APOTHEM * cos(degreesToRadians(-150))
    _REEF_FACE_7_Y = FieldConstants.REEF_CENTER_Y + REEF_APOTHEM * sin(degreesToRadians(-150))

    _REEF_FACE_9_X = FieldConstants.REEF_CENTER_X + REEF_APOTHEM * cos(degreesToRadians(-90))
    _REEF_FACE_9_Y = FieldConstants.REEF_CENTER_Y + REEF_APOTHEM * sin(degreesToRadians(-90))

    _REEF_FACE_10_X = FieldConstants.REEF_CENTER_X + REEF_APOTHEM * cos(degreesToRadians(-30))
    _REEF_FACE_10_Y = FieldConstants.REEF_CENTER_Y + REEF_APOTHEM * sin(degreesToRadians(-30))

    REEF_FACE_1 = center_relative_offset(_REEF_FACE_1_X, _REEF_FACE_1_Y)
    REEF_FACE_3 = center_relative_offset(_REEF_FACE_3_X, _REEF_FACE_3_Y)
    REEF_FACE_5 = center_relative_offset(_REEF_FACE_5_X, _REEF_FACE_5_Y)
    REEF_FACE_7 = center_relative_offset(_REEF_FACE_7_X, _REEF_FACE_7_Y)
    REEF_FACE_9 = center_relative_offset(_REEF_FACE_9_X, _REEF_FACE_9_Y)
    REEF_FACE_10 = center_relative_offset(_REEF_FACE_10_X, _REEF_FACE_10_Y)


    clockface = {
        1: REEF_FACE_1,
        3: REEF_FACE_3,
        5: REEF_FACE_5,
        7: REEF_FACE_7,
        9: REEF_FACE_9,
        10: REEF_FACE_10
    }
    if oclock not in clockface:
        raise ValueError("Invalid reef face [1, 3, 5, 7, 9, 10]")

    reef_face = clockface[oclock]
    reef_face_x = reef_face[0] + standoff * cos(degreesToRadians(oclock * 30))
    reef_face_y = reef_face[1] + standoff * sin(degreesToRadians(oclock * 30))

    # Now calculate the angle
    angles_for_clockface = {
        1: 45,
        3: 90,
        5: 135,
        7: -135,
        9: -90,
        10: -45
    }
    angle = angles_for_clockface[oclock] if facing_center else (180 + angles_for_clockface[oclock])

    return Pose2d(reef_face_x, reef_face_y, degreesToRadians(angle))