# Recipe for a new subsystem

## Overview

A subsystem is a logical group of hardware that works together to perform a specific function. 
For example, the `Drive` subsystem is responsible for controlling the robot's drive motors. The `Shooter` subsystem is 
responsible for the sensors and motors of the shooting subsystem.

## Steps for adding or changing a component in an existing subsystem

When you add a new component, you may or may not to create a new component in the `components` directory. Some hardware
pieces have such a simple interface that they don't need a separate component. For example, a gyro might be so simple 
and logic-free that you just read its value in your Drive subsystem. On the other hand, is a gyro really that 
straightforward? What if you want to reset it with a button press? What if you want to calibrate it dynamically? In that
case, you probably *do* want a separate component with the logic for those operations. 

This should hold the hardware interface for the component. If there is only a single piece of hardware (like a Gyro), 
then use the[Singleton pattern](../../SingletonPatternRecipe.md) to ensure that only one instance of the component is 
created. If there are multiples of the hardware component (like a swerve module) then use a normal `__init__` method.

Do *not* send Commands directly to components. Instead, send Commands to Subsystems. Subsystems are responsible for 
sending commands to components. This is because a subsystem is responsible for coordinating the actions of multiple
components. For example, the `Drive` subsystem is responsible for coordinating the actions of the four swerve modules.

So, even for something very straightforward like resetting a gyro, you should have a `ResetGyro` command that calls a 
`setZeroHeading()` method in your `DriveSubsystem` class (you wouldn't want to call the method `DriveSubsystem.resetGyro`
because that's a very "how" name. Instead, you want to call it `setZeroHeading` because that's a "what" name). The 
`setZeroHeading()` method would call the `reset()` method on the gyro component. It's fine for hardware components to 
have method names that are very "how" because they are the lowest level of abstraction.

## Steps for a **new** subsystem (rare)

1. Determine what the subsystem is responsible for. What is the subsystem's primary function? This should
be a noun that describes the subsystem's purpose. For example, the `Drive` subsystem is responsible for
... driving the robot. It contains the motors and sensors (`Gyro`) that control the robot's movement.

1. Create a new Python file in the `subsystems` directory. The file name should be the name of the subsystem in 
lowercase with `.py` appended to the end. For example, `subsystems/drive.py`.

1. Create a new Python class named after the subsystem in CamelCase. For example, `Drive`. This class should inherit 
from the `commands2.SubsystemBase` class.

1. In the class, implement the `__init__` method. In this method, you should create instance variables for the motors, 
sensors, and other components that the subsystem uses. You should also call the `super().__init__()` method. Create a
`logger` object using the `NTLoggerUtility` class. 

1. Create a new Command for diagnosing the subsystem. Use the [Command Recipe](../../NewCommandRecipe.md) to add a new 
command called `DiagnoseSubsystem`. This command should do nothing but communicate the subsystem's status. Most 
likely, this will use the `NTLoggerUtility` class to log the subsystem's status to NetworkTables (Table `subsystemLogs`, 
key `Diagnostics`).But perhaps there are on-robot LEDs that can be used to communicate the subsystem's status? 
**Important**: Design this command so that it can be run when the robot is disabled. That implies that you cannot 
programmatically move components. However, you can read sensors and attempt to communicate with hardware components. For 
instance, you could read the temperature of a motor or the voltage of a battery. If those function calls work, at least
you know that the hardware is connected and functioning. This command is not for validating logic. It is for verifying
only that the hardware is connected as expected and offering expected functionality.  

1. For complex subsystems, you might create a `DiagnoseWhileEnabled` command that runs while the robot is enabled. This 
command could move components and verify that they are working correctly. For instance, the `Drive` subsystem could 
align the swerve modules and verify that they are moving correctly. 