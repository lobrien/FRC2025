from dataclasses import dataclass


@dataclass(frozen=True)
class CoralConsts:

    CORAL_MOTOR = 33 # Motor ID number

    CORAL_STOP_CURRENT = 70 # TODO: Check what the AMP was when coral is all the way intaked 