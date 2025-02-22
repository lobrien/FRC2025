from dataclasses import dataclass

@dataclass(frozen=True)
class AlgaeConsts:

    ALGAE_MOTOR = 16 # Motor ID number

    ALGAE_STOP_CURRENT = 70 # TODO: Check what the AMP was when algae is all the way intaked 