from dataclasses import dataclass

@dataclass(frozen=True)
class VisionConsts:
    # Limelight pose in robot space
    # TODO: 2025-03-07: These values are placeholders and should be updated once limelight's position on robot is known.
    LIMELIGHT_X_OFFSET_INCHES = 0.0
    LIMELIGHT_Y_OFFSET_INCHES = 0.0
    LIMELIGHT_Z_OFFSET_INCHES = 0.0
