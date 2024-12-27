
# Install prerequisites 

- [Install Python 3.12 on your development computer(s)](https://www.python.org/downloads/)
- [Install FRC Game and RobotPy on your dev computer(s)](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html)
- [Install Poetry on your dev machines](https://python-poetry.org/docs/)

# Initialize Python dependencies

- Run `poetry install`. This uses the `pyproject.toml` project configuration file and the `poetry.lock` file to determine what packages will be downloaded (by `pip` behind the scenes). 

# Check Installation

- Open a terminal 
- Switch to this repository's root directory
- Run `poetry shell` to activate the Poetry environment, **and then**
- Run `robotpy --main src/hello_robot/hello_robot.py sim`

You should see the simulation window:

![image](media/simulation_gui.png)

And in your console you should see something like:

```
10:01:35:647 INFO    : faulthandler        : registered SIGUSR2 for PID 2744
10:01:35:649 INFO    : halsim_gui          : WPILib HAL Simulation 2024.3.2.1
HAL Extensions: Attempting to load: libhalsim_gui
Simulator GUI Initializing.
Simulator GUI Initialized!
HAL Extensions: Successfully loaded extension
10:01:35:767 WARNING : pyfrc.physics       : Cannot enable physics support, /Users/lobrien/Documents/src/FRC/FRC2025/src/hello_robot/physics.py not found
10:01:35:768 INFO    : wpilib              : RobotPy version 2024.3.2.2
10:01:35:768 INFO    : wpilib              : WPILib version 2024.3.2.1
10:01:35:768 INFO    : wpilib              : Running with simulated HAL.
10:01:35:770 INFO    : nt                  : could not open persistent file 'networktables.json': No such file or directory (this can be ignored if you aren't expecting persistent values)
10:01:35:771 INFO    : nt                  : Listening on NT3 port 1735, NT4 port 5810
Not loading CameraServerShared
Success

********** Robot program startup complete **********
2024-12-24 10:01:35.791 Python[2744:71983881] +[IMKClient subclass]: chose IMKClient_Modern
2024-12-24 10:01:35.791 Python[2744:71983881] +[IMKInputSession subclass]: chose IMKInputSession_Modern
Default DisabledPeriodic() method... Override me!
Default RobotPeriodic() method... Override me!
Default SimulationPeriodic() method... Override me!

```
Close the simulation GUI window to stop the simulation. Congratulations! You have successfully installed the FRC tools and dependencies for this project.

### Troubleshooting Installation

If you do not see the simulation window, or if you see an error message, please check the following:

- Did you run `poetry shell` before running `robotpy`?

If you see an error message like `command not found: robotpy`, then you may need to install the `robotpy` command line tool. You can do this by running `poetry install` again.

- Did you run `robotpy` from the root directory of this repository?

If you see an error message like `ERROR: /src/hello_robot/hello_robot.py does not exist`, then you may not be in the root directory of this repository.

    - If you are running a Windows Command or Powershell prompt, check which directory you are in by running `dir`. 
    - If you are running a Unix shell, check which directory you are in by running `pwd`.

- Did you see any error messages when you ran `poetry install`?

If you see an error message like `ERROR: Could not find a version that satisfies the requirement ...`, then you may have a network connectivity issue. 

    - Check your network connection
    - Run `poetry install` again

- Did you see any error messages when you ran `poetry shell`?

If you see an error message like `ERROR: The virtual environment was not created successfully because ensurepip is not available.`, then you may need to install `ensurepip` manually. 

    - Run `poetry shell` again
    - Run `python -m ensurepip`

- Did you see any error messages when you ran `robotpy`?

You should install the `robotpy` command-line tool while installing the FRC wpilib tools on this machine. 

