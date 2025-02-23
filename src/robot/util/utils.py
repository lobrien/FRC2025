import wpilib


def is_red_alliance() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed