from dataclasses import dataclass

from constants.new_types import inches, degrees


@dataclass(frozen=True)
class VisionConsts:
    # Limelight pose in robot space
    # TODO: 2025-03-07: These values are placeholders and should be updated once limelight's position on robot is known.
    LIMELIGHT_X_OFFSET_INCHES = 0.0
    LIMELIGHT_Y_OFFSET_INCHES = 0.0
    LIMELIGHT_Z_OFFSET_INCHES = 0.0

    # "Close enough" constants for alignment
    ALIGN_TRANSFORM_TOLERANCE = inches(1.0)
    ALIGN_ROTATION_TOLERANCE = degrees(5.0)