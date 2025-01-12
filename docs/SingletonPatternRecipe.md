from util.ntloggerutility import NTLoggerUtility

# Factory Pattern recipe

## Problem

You have a sensor or piece of hardware that is used in multiple subsystems. You want to ensure that only one instance of the sensor is created.

## Solution

Use the **Singleton pattern** to create a single instance of the sensor and pass it to the subsystems that need it.

## Example

Let's say you have a `Lidar` sensor that is used in both the `Drive` and `Vision` subsystems. You want to ensure that only one instance of the `Lidar` class is created.

In your `Lidar` class, create a "static" method that returns the single instance of the `Lidar` class.
To make a method "static" in Python, use the `@staticmethod` decorator. 

The **Singleton pattern**:
- Use a private class variable named `_instance` to store the single instance of the class. 
- Name the variable `_instance` (the underscore indicates that it is a private variable).
- Use a static method named `get_instance` to return the single instance of the class.
- Add an `__init__` method that raises a `RuntimeError` to prevent the class from being instantiated directly. 
- Add a `logger` to the `_instance` for writing to Network Tables. 

```python   
from util.ntloggerutility import NTLoggerUtility

class Lidar:
    _instance = None

    @staticmethod
    def get_instance():
        if Lidar._instance is None:
            Lidar._instance = Lidar()
        Lidar._instance.logger = NTLoggerUtility("LidarLogs")
        return Lidar._instance

    def __init__(self):
        self.logger.critical("Attempt to initialize Lidar. Use `Lidar.get_instance()` instead")
        raise RuntimeError("Call Lidar.get_instance() instead")
```

To use the `Lidar` class in the `Drive` and `Vision` subsystems, call the `get_instance` method to get the single instance of the `Lidar` class:

```python
from components.lidar import Lidar

reference_to_lidar = Lidar.get_instance()
```
Notice that static methods do not take `self` as an argument. Instead,
you call the static method on the class itself, not on an instance of the class. So `Lidar.get_instance()` will return the single instance of the `Lidar` class.

You don't have to name the variable `reference_to_lidar`. You can name it whatever you want (probably `lidar`). The important thing is that you use the `get_instance` method to get the single instance of the `Lidar` class. 