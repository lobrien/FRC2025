# Where should I put...?

- **A new command?** `commands/{subsystem/` Use the [Command Recipe](../../NewCommandRecipe.md) to add a new command.
- **A brand new subsystem?** Put the subsystem class in `subsystems/`. Write a testing class for it and put that in `tests`.
You'll also need to create an instance of the object in the `RobotSystems` class in `RobotSystems.py`. Use the [Subsystem Recipe](../../NewSubsystemRecipe.md).
- **A new sensor or piece of hardare?** `components/`. If it's only used in a single subsystem, create a normal `__init__`
If it's used in multiple subsystems, use the [Singleton pattern](../../SingletonPatternRecipe.md) so that only one 
instance of the sensor is created.
- **Brand new behavior?** If it's a behavior that reads or writes to and from hardware, it should be a `Command`. If 
it's a behavior that doesn't interact with hardware, it should be a utility function in `utilities/`.

# Sharkbot Code Organization

The 2025 Sharkbot is a "command-based" robot. 

## Sharkbot Class

The "entry point" for the robot is the `Sharkbot` class. This class will change very rarely, as it mostly exists to hold the `RobotSystems` container class.

## RobotSystems Class
The `RobotSystems` class is a container for all of the robot's subsystems. It is responsible for creating the subsystems. This class will only change when subsystems are added or removed.

## Subsystems
The `subsystems` directory contains the definitions for the robots subsystem. A subsystem is a logical group of hardware that works together to perform a specific function. For example, the `Drive` subsystem is responsible for controlling the robot's drive motors. The `Vision` subsystem is responsible for interfacing with the camera.

## Commands

The `commands` directory contains the definitions for the robot's commands. A command is a small piece of code that performs a specific task. For example, the `FireCommand` class is responsible for the sequence of events that occur when the robot's `Shooter` subsystem fires a ball. In a simple case, maybe that's just opening a gate, waiting for the ball to clear, and closing the gate. 

## Components

The `components` directory contains the definitions for the robot's components. A component is a small piece of code that performs a specific task. For example, the `SwerveModule` class is responsible for controlling a single swerve module on the robot.

## Constants

The `constants` directory contains the definitions for the robot's constants. Constants are values that are used throughout the robot code. For example, the `DriveConstants` class contains the maximum speed of the robot's drive motors.

## Simulation

The `simulation` directory contains the definitions for the robot's simulation. The simulation is a way to test the robot code without having the physical robot. The simulation code should be as close to the real robot code as possible. (Note: This approach may be a little too advanced for the 2025 season)

## Tests

The `tests` directory contains the definitions for the robot's tests. Tests are small pieces of code that verify that the robot code works as expected. For example, the `DriveTest` class might verify that the robot drives straight when the `Drive` subsystem is commanded to drive straight.

## Utilities

The `utilities` directory contains the definitions for the robot's utilities. Utilities are small pieces of code that are used throughout the robot code. For example, the `MathUtils` class might contain utility functions for doing math operations.