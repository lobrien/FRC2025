from dataclasses import dataclass

from constants.new_types import inches


@dataclass(frozen=True)
class ElevatorConstants:
    ELEVATOR_MOTOR = 14  # Motor ID

    STOP_CURRENT = 70 #TODO: Update

    # TODO: Update height offset, and positions

    # Mechanical constants
    GEAR_RATIO: float = 1.0
    HEIGHT_OFFSET: float = (
        10.5  # Height of the second stage's lower crosspiece (top surface), in inches.
    )
    SCREW_INCHES_PER_ROT = 0.25

    # Heights in inches (lowest is 10.5, highest is ~55.5)  #Change names of the different heights when writing the official code
    HOME: inches = 10.5  # Elevator at its lowest position.
    MID: inches = 34.5

    # TODO: These need to be measured and updated.
    SPROCKET_CIRC = 42 * 5 / 25.4  # From Mako TODO: Measure this
    RIG = 2.0  # From Mako: rig design causes the second stage to move at 2x the rate of first stage.
