from dataclasses import dataclass


@dataclass(frozen=True)
class CoralConsts:

    CORAL_MOTOR = 33 # Motor ID number

    CORAL_STOP_CURRENT = 30 #electrical current that happens when the coral is fully intaked  