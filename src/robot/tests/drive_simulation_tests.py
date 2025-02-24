import math
import unittest

import numpy as np
from wpilib import RobotBase
from wpimath._controls._controls.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModulePosition, ChassisSpeeds
from wpimath.units import metersToInches, inchesToMeters

from constants.new_types import inches_per_second
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.swerve_module import SwerveModule


class DriveSimulationTests(unittest.TestCase):
    def setUp(self):
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

        initial_pose = Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45.0))
        self.drive_subsystem.gyro.set_yaw(45.0)
        self.drive_subsystem.odometry = SwerveDrive4PoseEstimator(
            kinematics=self.drive_subsystem.kinematics,
            gyroAngle=self.drive_subsystem.get_heading_rotation2d(),
            modulePositions=[module.get_position() for module in self.drive_subsystem.modules],
            initialPose=initial_pose,
            stateStdDevs=np.array([0.1, 0.1, 0.1]),
            visionMeasurementStdDevs=np.array([0.9, 0.9, 0.9])
        )

        # Print initial state
        print(f"\nInitial state:")
        print(f"Robot heading: {self.drive_subsystem.get_heading_degrees():.2f} degrees")
        pose = self.drive_subsystem.odometry.getEstimatedPosition()
        print(f"Robot pose: X={pose.X():.2f}, Y={pose.Y():.2f}, Rotation={pose.rotation().degrees():.2f}")

        # Drive forward in robot frame for 1 second
        forward_speed = 36.0  # inches/second

        # Simulate 50 cycles (20ms each = 1 second)
        for i in range(50):
            # Command robot to drive forward
            self.drive_subsystem.drive(
                x_speed_inches_per_second=forward_speed,  # Forward
                y_speed_inches_per_second=0.0,  # No sideways motion
                rot_speed_degrees_per_second=0.0  # No rotation
            )

            # Run periodic updates
            self.drive_subsystem.periodic()

            # Print state every 10 cycles
            if i % 10 == 0:
                print(f"\nCycle {i}:")
                print(f"Robot heading: {self.drive_subsystem.get_heading_degrees():.2f} degrees")
                pose = self.drive_subsystem.odometry.getEstimatedPosition()
                print(f"Robot pose: X={pose.X():.2f}, Y={pose.Y():.2f}, "
                      f"Rotation={pose.rotation().degrees():.2f}")
                for j, module in enumerate(self.drive_subsystem.modules):
                    pos = module.get_position()
                    print(f"Module {j}: pos={metersToInches(pos.distance):.2f} in, "
                          f"angle={pos.angle.degrees():.2f} deg")

        # Get final state
        final_pose = self.drive_subsystem.odometry.getEstimatedPosition()
        print(f"\nFinal state:")
        print(f"Robot heading: {self.drive_subsystem.get_heading_degrees():.2f} degrees")
        print(f"Robot pose: X={final_pose.X():.2f}, Y={final_pose.Y():.2f}, "
              f"Rotation={final_pose.rotation().degrees():.2f}")

        # Calculate expected position:
        # Starting at (2,3) at 45°, moving forward 12 inches/sec for 1 second
        # Robot-relative forward motion gets split into X and Y components in field frame
        distance_moved = inchesToMeters(36.0)
        expected_x = 2.0 + (distance_moved * np.cos(np.pi / 4))
        expected_y = 3.0 + (distance_moved * np.sin(np.pi / 4))

        print(f"Expected position: X={expected_x:.2f}, Y={expected_y:.2f}")

        # Verify position and heading
        self.assertAlmostEqual(final_pose.X(), expected_x, delta=0.2)
        self.assertAlmostEqual(final_pose.Y(), expected_y, delta=0.2)
        self.assertAlmostEqual(final_pose.rotation().degrees(), 45.0, delta=0.1)


    def test_simulate_position_single_cycle(self):
        # Create a SwerveModule with a fixed offset and set the simulated angle to 45°.
        module = SwerveModule(
            name="test",
            drive_motor_bus_id = 0,
            turn_motor_bus_id = 0,
            cancoder_bus_id = 0,
            offset_rotations = 0,
            offset_translation = (1,2)
        )
        module._simulated_angle = 45.0  # Force the wheel to 45°
        module._simulated_drive_position = 0.0

        # Call simulate_position with 36 in/s forward, 0 sideways, 0 rotation.
        pos = module.simulate_position(36.0, 0.0, 0.0)

        # Expected effective speed: 36 in/s = 0.9144 m/s; projected effective speed = 0.9144 * cos(45°)
        expected_effective_speed = 0.9144 * math.cos(math.radians(45))
        expected_delta = expected_effective_speed * 0.02  # in meters

        # Convert the integrated drive position back to meters:
        integrated_distance = inchesToMeters(module._simulated_drive_position)

        assert math.isclose(integrated_distance, expected_delta, rel_tol=0.05), \
            f"Expected delta {expected_delta:.3f} m, got {integrated_distance:.3f} m"

    def test_simulate_gyro_no_rotation(self):
        drive_subsystem = DriveSubsystem()
        drive_subsystem.gyro = FakeGyro()
        # Force simulation mode if necessary (depending on your framework)
        # For example, you might need to set a flag or use a simulated gyro.
        drive_subsystem.gyro.set_yaw(45.0)
        # Immediately read back the gyro yaw to confirm it was set
        initial_yaw = drive_subsystem.gyro.get_yaw().value
        assert math.isclose(initial_yaw, 45.0, rel_tol=0.01), f"Initial yaw not set correctly: {initial_yaw}"

        # Call the gyro simulation with zero rotational speed
        heading = drive_subsystem._simulate_gyro(0.0)
        # Expect the heading to remain unchanged.
        assert math.isclose(heading, 45.0, rel_tol=0.01), f"Expected heading 45°, got {heading}"

    def test_simulate_gyro_with_rotation(self):
        drive_subsystem = DriveSubsystem()
        drive_subsystem.gyro = FakeGyro()
        drive_subsystem.gyro.set_yaw(45.0)
        initial_yaw = drive_subsystem.gyro.get_yaw().value
        # Command a small rotation (e.g., 10 deg/s) for one period (20 ms).
        rot_speed = 10.0  # deg/s
        period = 0.02  # seconds
        expected_delta = rot_speed * period  # 0.2 deg expected change

        heading = drive_subsystem._simulate_gyro(rot_speed)
        expected_heading = initial_yaw + expected_delta
        assert math.isclose(heading, expected_heading, rel_tol=0.01), \
            f"Expected heading {expected_heading}°, got {heading}°"

    def test_periodic_updates(self):
        drive_subsystem = DriveSubsystem()
        # Set simulation speeds (for example, 36 in/s forward, 0 lateral, 0 rotation)
        drive_subsystem.last_x_speed = 36.0
        drive_subsystem.last_y_speed = 0.0
        drive_subsystem.last_rot_speed = 0.0

        # Set an initial gyro yaw
        drive_subsystem.gyro.set_yaw(45.0)

        # Ensure odometry is initialized
        drive_subsystem.odometry = SwerveDrive4PoseEstimator(
            kinematics=drive_subsystem.kinematics,
            gyroAngle=drive_subsystem.get_heading_rotation2d(),
            modulePositions=[module.get_position() for module in drive_subsystem.modules],
            initialPose=Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45.0)),
            stateStdDevs=np.array([0.1, 0.1, 0.1]),
            visionMeasurementStdDevs=np.array([0.9, 0.9, 0.9])
        )

        drive_subsystem.periodic()

        # Verify that each module returns a valid SwerveModulePosition
        for module in drive_subsystem.modules:
            pos = module.get_position()
            assert pos is not None, "Module position is None"

        # You could also retrieve the estimated pose and assert it is not the initial one.
        estimated_pose = drive_subsystem.odometry.getEstimatedPosition()
        assert estimated_pose is not None, "Odometry did not update"

    #@unittest.expectedFailure # Expect failure because no real gyro during sim
    # Because I now always use fake gyro, this should pass again
    def test_gyro_set_get_yaw(self):
        drive_subsystem = DriveSubsystem()
        drive_subsystem.gyro = FakeGyro()
        # Set the yaw to 45.0
        drive_subsystem.gyro.set_yaw(45.0)
        # Immediately retrieve the value
        current_yaw = drive_subsystem.gyro.get_yaw().value
        # Expect it to be 45.0
        assert math.isclose(current_yaw, 45.0, rel_tol=0.01), \
            f"Expected gyro yaw 45.0, but got {current_yaw}"

    def test_robot_is_simulation(self):
        assert RobotBase.isSimulation(), "Expected RobotBase.isSimulation() to be True in simulation mode"

    def test_fake_gyro_set_get_yaw(self):
        fake_gyro = FakeGyro()
        fake_gyro.set_yaw(45.0)
        current_yaw = fake_gyro.get_yaw().value
        assert math.isclose(current_yaw, 45.0, rel_tol=0.01), \
            f"Expected fake gyro yaw 45.0, but got {current_yaw}"

    def test_odometry_update_simulation(self):
        # Run the periodic update for 50 cycles (1 second).
        for i in range(50):
            self.drive_subsystem.drive(x_speed_inches_per_second=36.0,
                                       y_speed_inches_per_second=0.0,
                                       rot_speed_degrees_per_second=0.0)
            self.drive_subsystem.periodic()

        final_pose = self.drive_subsystem.odometry.getEstimatedPosition()

        # Calculation:
        # 36 in/s = 0.9144 m/s. With a desired wheel angle of 45°,
        # effective speed = 0.9144 * cos(45°) ≈ 0.9144 * 0.7071 ≈ 0.647 m/s.
        # Over 1 second, displacement = 0.647 m.
        # Thus, expected final pose (in meters) is approximately:
        expected_x = 2.0 + 0.647  # ≈ 2.647 m
        expected_y = 3.0 + 0.647  # ≈ 3.647 m
        expected_heading = 45.0  # unchanged

        self.assertAlmostEqual(final_pose.X(), expected_x, delta=0.05,
                               msg=f"Expected X ≈ {expected_x:.3f}, got {final_pose.X():.3f}")
        self.assertAlmostEqual(final_pose.Y(), expected_y, delta=0.05,
                               msg=f"Expected Y ≈ {expected_y:.3f}, got {final_pose.Y():.3f}")
        self.assertAlmostEqual(final_pose.rotation().degrees(), expected_heading, delta=0.5,
                               msg=f"Expected heading {expected_heading}, got {final_pose.rotation().degrees()}")

    def test_odometry_update_simulation(self):
        # For this test only, override the gyro with a FakeGyro that works correctly.
        self.drive_subsystem.gyro = FakeGyro()
        self.drive_subsystem.gyro.set_yaw(45.0)

        # Set the last commanded speeds: 36 in/s forward, 0 lateral, 0 rotation.
        self.drive_subsystem.last_x_speed = 36.0
        self.drive_subsystem.last_y_speed = 0.0
        self.drive_subsystem.last_rot_speed = 0.0

        # Reinitialize odometry with an initial pose of (2.0, 3.0, 45°).
        initial_pose = Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45.0))
        self.drive_subsystem.odometry = SwerveDrive4PoseEstimator(
            kinematics=self.drive_subsystem.kinematics,
            gyroAngle=self.drive_subsystem.get_heading_rotation2d(),
            modulePositions=[module.get_position() for module in self.drive_subsystem.modules],
            initialPose=initial_pose,
            stateStdDevs=np.array([0.1, 0.1, 0.1]),
            visionMeasurementStdDevs=np.array([0.9, 0.9, 0.9])
        )

        # Run periodic updates for 50 cycles (1 second).
        for i in range(50):
            self.drive_subsystem.drive(
                x_speed_inches_per_second=36.0,
                y_speed_inches_per_second=0.0,
                rot_speed_degrees_per_second=0.0
            )
            self.drive_subsystem.periodic()

        final_pose = self.drive_subsystem.odometry.getEstimatedPosition()

        # Calculate expected displacement:
        # 36 in/s = 36 * 0.0254 ≈ 0.9144 m/s.
        # With a wheel (and robot) angle of 45°, the effective speed along the wheel is:
        # 0.9144 * cos(45°) ≈ 0.9144 * 0.7071 ≈ 0.647 m/s.
        # Over 1 second, the displacement is about 0.647 m.
        expected_x = 2.0 + 0.647  # ≈ 2.647 m
        expected_y = 3.0 + 0.647  # ≈ 3.647 m
        expected_heading = 45.0  # unchanged

        self.assertAlmostEqual(final_pose.X(), expected_x, delta=0.05,
                               msg=f"Expected X ≈ {expected_x:.3f}, got {final_pose.X():.3f}")
        self.assertAlmostEqual(final_pose.Y(), expected_y, delta=0.05,
                               msg=f"Expected Y ≈ {expected_y:.3f}, got {final_pose.Y():.3f}")
        self.assertAlmostEqual(final_pose.rotation().degrees(), expected_heading, delta=0.5,
                               msg=f"Expected heading {expected_heading}, got {final_pose.rotation().degrees()}")

    def test_module_simulated_position_projection(self):
        module = SwerveModule("TestModule", 1, 2, 3, 0.0, (0.0, 0.0))
        # Set the simulated steering angle to 45°
        module._simulated_angle = 45.0
        module._simulated_drive_position = 0.0
        pos = module.simulate_position(36.0, 0.0, 0.0)
        expected_delta = 0.9144 * math.cos(math.radians(45.0)) * 0.02  # in meters
        self.assertAlmostEqual(pos.distance, expected_delta, delta=0.001,
                               msg=f"Expected module travel {expected_delta:.3f} m, got {pos.distance:.3f} m")

    def test_module_simulation_multiple_cycles(self):
        module = SwerveModule("TestModule", 1, 2, 3, 0.0, (0.0, 0.0))
        module._simulated_angle = 45.0
        module._simulated_drive_position = 0.0
        cycles = 50
        for _ in range(cycles):
            module.simulate_position(36.0, 0.0, 0.0)
        # Calculate expected integrated distance over 50 cycles:
        # 36 in/s = 0.9144 m/s. Effective speed = 0.9144 * cos(45°) ≈ 0.647 m/s.
        # Each cycle is 0.02 seconds, so total displacement ≈ 0.647 * 50 * 0.02.
        expected_distance = 0.647 * cycles * 0.02  # in meters
        actual_distance = inchesToMeters(module._simulated_drive_position)
        self.assertAlmostEqual(actual_distance, expected_distance, delta=0.01,
                               msg=f"Expected integrated distance ≈ {expected_distance:.3f} m, got {actual_distance:.3f} m")

    def test_get_position_returns_correct_angle(self):
        module = SwerveModule("TestModule", 1, 2, 3, 0.0, (0.0, 0.0))
        module._simulated_angle = 45.0
        module._simulated_drive_position = 10.0  # in inches, for instance
        pos = module.get_position()
        self.assertAlmostEqual(pos.angle.degrees(), 45.0, delta=0.1,
                               msg=f"Expected module angle 45°, got {pos.angle.degrees()}°")

    def test_module_translations(self):
        # Retrieve the module translations from the drive subsystem.
        translations = self.drive_subsystem._get_module_translations()
        # For example, if your robot has front modules at (wheelbase_half_length, ±track_half_width),
        # then check that the front-right translation has positive x and negative y (if that is your convention).
        fr = translations[0]
        self.assertGreater(fr.X(), 0, "Front-right module translation X should be positive")
        self.assertLess(fr.Y(), 0, "Front-right module translation Y should be negative")

    def test_odometry_update_diagnostics(self):
        drive_subsystem = DriveSubsystem()
        drive_subsystem.gyro = FakeGyro()
        drive_subsystem.gyro.set_yaw(45.0)
        drive_subsystem.last_x_speed = 36.0
        drive_subsystem.last_y_speed = 0.0
        drive_subsystem.last_rot_speed = 0.0

        initial_pose = Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45.0))
        drive_subsystem.odometry = SwerveDrive4PoseEstimator(
            kinematics=self.drive_subsystem.kinematics,
            gyroAngle=drive_subsystem.get_heading_rotation2d(),
            modulePositions=[module.get_position() for module in drive_subsystem.modules],
            initialPose=initial_pose,
            stateStdDevs=np.array([0.1, 0.1, 0.1]),
            visionMeasurementStdDevs=np.array([0.9, 0.9, 0.9])
        )

        # Run one cycle
        drive_subsystem.drive(x_speed_inches_per_second=36.0,
                              y_speed_inches_per_second=0.0,
                              rot_speed_degrees_per_second=0.0)
        drive_subsystem.periodic()

        # Capture gyro and module positions
        heading = drive_subsystem.get_heading_degrees()
        module_positions = [module.get_position() for module in drive_subsystem.modules]
        # Print them (or assert expected values)
        print("Gyro heading after one cycle:", heading)
        for i, pos in enumerate(module_positions):
            print(f"Module {i} position: distance={pos.distance:.3f} m, angle={pos.angle.degrees():.1f} deg")

        # Then update odometry and check the new pose.
        new_pose = drive_subsystem.odometry.getEstimatedPosition()
        print("Updated pose:", new_pose)

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
