from dataclasses import dataclass

@dataclass(frozen=True)
class FieldPositions:
    speaker_x_red = 8.31
    speaker_x_blue = -8.31
    speaker_y = 1.44

    desired_x_for_autonomous_driving_red = 5.5
    desired_x_for_autonomous_driving_blue = -5.5

