from dataclasses import dataclass

from constants.new_types import inches


@dataclass(frozen=True)
class FieldConstants:
    FIELD_LENGTH : inches = 689.5 # From field diagram
    FIELD_WIDTH : inches = 317

    FIELD_HALF_LENGTH : inches = FIELD_LENGTH / 2
    FIELD_HALF_WIDTH : inches = FIELD_WIDTH / 2

    AUTO_START_X : inches = FIELD_HALF_LENGTH + 24.0 # Robot origin on edge of starting area
    AUTO_START_Y : inches = FIELD_HALF_WIDTH

