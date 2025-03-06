import math
import unittest
from unittest.mock import patch

import numpy as np
import wpilib
from wpilib import RobotBase
from wpimath._controls._controls.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModulePosition, ChassisSpeeds
from wpimath.units import metersToInches, inchesToMeters

from constants.new_types import inches_per_second, degrees_per_second
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.swerve_module import SwerveModule


class DriveSimulationTests(unittest.TestCase):
    def setUp(self):
        RobotBase.isSimulation = staticmethod(lambda: True)

        self.drive_subsystem = DriveSubsystem()
        # Inject the FakeGyro into the subsystem for testing
        self.drive_subsystem.gyro = FakeGyro()
        self.drive_subsystem.gyro.set_yaw(45.0)
        assert self.drive_subsystem.gyro.get_yaw().value == 45.0, "Failed to set gyro yaw"

        for module in self.drive_subsystem.modules:
            module._simulated_angle = 45.0
            module._simulated_drive_position = 0.0

    def tearDown(self):
        pass

    def test_can_create(self):
        assert self.drive_subsystem is not None

    def test_initial_setup(self):
        # Verify that the DriveSubsystem is created
        self.assertIsNotNone(self.drive_subsystem)

        # Verify the initial pose of the robot
        initial_pose = self.drive_subsystem.odometry.getEstimatedPosition()
        self.assertEqual(initial_pose, Pose2d())  # Should start at (0, 0) with 0 rotation


    def test_initial_setup_non_zero_pose(self):
        # Set a non-zero initial pose
        initial_pose = Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45.0))  # X=2.0, Y=3.0, Rotation=45 degrees

        # Reinitialize the odometry with the non-zero initial pose
        self.drive_subsystem.odometry = SwerveDrive4PoseEstimator(
            kinematics=self.drive_subsystem.kinematics,
            gyroAngle=self.drive_subsystem.get_heading_rotation2d(),
            modulePositions=[module.get_position() for module in self.drive_subsystem.modules],
            initialPose=initial_pose,  # Set the initial pose here
            stateStdDevs=np.array([0.1, 0.1, 0.1]),  # X, Y, rotation standard deviations
            visionMeasurementStdDevs=np.array([0.9, 0.9, 0.9])  # Vision measurement uncertainties
        )

        # Verify that the odometry reflects the non-zero initial pose
        estimated_pose = self.drive_subsystem.odometry.getEstimatedPosition()
        self.assertAlmostEqual(estimated_pose.X(), 2.0, delta=0.01)  # X should be 2.0
        self.assertAlmostEqual(estimated_pose.Y(), 3.0, delta=0.01)  # Y should be 3.0
        self.assertAlmostEqual(estimated_pose.rotation().degrees(), 45.0, delta=0.01)  # Rotation should be 45 degrees

    def test_module_positions(self):
        # Get the initial positions of all modules
        module_positions = [module.get_position() for module in self.drive_subsystem.modules]

        # Verify that all modules report valid positions
        for position in module_positions:
            self.assertIsInstance(position, SwerveModulePosition)
            self.assertIsInstance(position.distance, float)
            self.assertIsInstance(position.angle, Rotation2d)

        # Verify that the odometry is updated with the module positions
        self.drive_subsystem.odometry.update(
            self.drive_subsystem.get_heading_rotation2d(),
            module_positions
        )
        updated_pose = self.drive_subsystem.odometry.getEstimatedPosition()
        self.assertEqual(updated_pose, Pose2d())  # Should still be at (0, 0) with 0 rotation

    def test_module_positions_non_zero_pose(self):
        """Test moving the robot from a non-zero starting pose by driving forward."""

        # Initialize robot at a non-zero pose
        initial_pose = Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45.0))

        # Set both the gyro and sim_pose to match
        self.drive_subsystem.gyro.set_yaw(45.0)
        self.drive_subsystem.sim_pose = initial_pose

        # Reset the odometry with the initial pose
        self.drive_subsystem.odometry = SwerveDrive4PoseEstimator(
            kinematics=self.drive_subsystem.kinematics,
            gyroAngle=self.drive_subsystem.get_heading_rotation2d(),
            modulePositions=[module.get_position() for module in self.drive_subsystem.modules],
            initialPose=initial_pose,
            stateStdDevs=np.array([0.1, 0.1, 0.1]),
            visionMeasurementStdDevs=np.array([0.9, 0.9, 0.9])
        )

        # Reset the simulation timer
        self.drive_subsystem.prev_sim_time = wpilib.Timer.getFPGATimestamp()

        # Print initial state
        print(f"\nInitial state:")
        print(f"Robot heading: {self.drive_subsystem.get_heading_degrees():.2f} degrees")
        pose = self.drive_subsystem.get_pose()
        print(f"Robot pose: X={pose.X():.2f}, Y={pose.Y():.2f}, Rotation={pose.rotation().degrees():.2f}")

        # Drive forward in robot frame
        forward_speed = inches_per_second(36.0)  # inches/second

        # First, check how ChassisSpeeds are being converted - do this before any simulation
        # This will help diagnose field vs. robot relative issues
        chassis_speeds = self.drive_subsystem._get_chassis_speeds(
            forward_speed, inches_per_second(0.0), degrees_per_second(0.0)
        )
        print(f"\nDiagnostics - Chassis speeds:")
        print(
            f"vx: {chassis_speeds.vx:.4f} m/s, vy: {chassis_speeds.vy:.4f} m/s, omega: {chassis_speeds.omega:.4f} rad/s")

        # Check what module states are being commanded - before any simulation
        module_states = self.drive_subsystem._speeds_to_states(
            forward_speed, inches_per_second(0.0), degrees_per_second(0.0)
        )
        print("\nDiagnostics - Commanded module states:")
        for i, state in enumerate(module_states):
            print(f"Module {i}: speed={metersToInches(state.speed):.2f} in/s, angle={state.angle.degrees():.2f} deg")

        # Now run the simulation
        cycle_time = 0.02  # 20ms per cycle
        for i in range(50):
            # Command robot to drive forward (in robot frame)
            self.drive_subsystem.drive(
                x_speed_inches_per_second=forward_speed,  # Forward
                y_speed_inches_per_second=inches_per_second(0.0),  # No sideways motion
                rot_speed_degrees_per_second=degrees_per_second(0.0)  # No rotation
            )

            # Advance simulation time
            mock_time = self.drive_subsystem.prev_sim_time + cycle_time
            with patch('wpilib.Timer.getFPGATimestamp', return_value=mock_time):
                # Run simulation updates FIRST, then periodic updates
                self.drive_subsystem.simulationPeriodic()
                self.drive_subsystem.periodic()

            # Print state every 10 cycles
            if i % 10 == 0:
                print(f"\nCycle {i}:")
                print(f"Robot heading: {self.drive_subsystem.get_heading_degrees():.2f} degrees")
                pose = self.drive_subsystem.get_pose()
                print(f"Robot pose: X={pose.X():.2f}, Y={pose.Y():.2f}, Rotation={pose.rotation().degrees():.2f}")
                for j, module in enumerate(self.drive_subsystem.modules):
                    pos = module.get_position()
                    state = module.get_state()
                    print(
                        f"Module {j}: pos={metersToInches(pos.distance):.2f} in, angle={pos.angle.degrees():.2f} deg, speed={metersToInches(state.speed):.2f} in/s")

        # ... rest of your test remains the same

class FakeGyro:
    def __init__(self):
        self._yaw = 0.0

    def set_yaw(self, yaw: float):
        self._yaw = yaw

    def get_yaw(self):
        # Return an object with a 'value' attribute.
        from collections import namedtuple
        Yaw = namedtuple("Yaw", "value")
        return Yaw(self._yaw)
