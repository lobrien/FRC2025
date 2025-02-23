from dataclasses import dataclass


@dataclass(frozen=True)
class AutoConsts:

    FORWARD = 1
    SIDE_STEP = 2
    SEQUENCE = 3
    TRAJECTORY = 4