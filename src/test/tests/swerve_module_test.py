import unittest

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

from constants.driveconstants import DriveConstants
from subsystems.swerve_module import SwerveModule

class SwerveModuleTest(unittest.TestCase):
    def test_get_turn_angle_degrees(self):
        test_module = SwerveModule("TestModule", 1, 2, 3, offset_rotations=0)
        self.assertIsNotNone(test_module)\

        angles_for_positions = {
            0 : 0,
            0.25 : 90,
            0.49999 : 179.9964,
            0.5 : 180,
            0.50001 : -179.9964,
            0.75 : -90,
        }
        for rotation, angle in angles_for_positions.items():
            # Mock the can_coder
            test_module._get_can_coder_pos_normalized = lambda: rotation
            angle_in_degrees = test_module.get_turn_angle_degrees()
            self.assertAlmostEqual(angle, angle_in_degrees)

    def test_get_turn_angle_degrees_with_rotational_offset(self):
        # Test with different rotational offsets
        test_cases = [
            # format: (offset_rotations, cancoder_reading, expected_angle)
            (0, 0.0, 0),  # No offset case
            (0, 0.25, 90),  # No offset, quarter turn
            (0, 0.5, 180),  # No offset, half turn
            (0, 0.75, -90),  # No offset, three-quarter turn

            (0.25, 0.25, 0),  # 90° offset (0.25), reading at offset position
            (0.25, 0.5, 90),  # 90° offset, quarter turn from offset
            (0.25, 0.75, 180),  # 90° offset, half turn from offset
            (0.25, 0.0, -90),  # 90° offset, three-quarter turn from offset

            (0.5, 0.5, 0),  # 180° offset (0.5), reading at offset position
            (0.5, 0.75, 90),  # 180° offset, quarter turn from offset
            (0.5, 0.0, 180),  # 180° offset, half turn from offset
            (0.5, 0.25, -90),  # 180° offset, three-quarter turn from offset

            (0.75, 0.75, 0),  # -90° offset (0.75), reading at offset position
            (0.75, 0.0, 90),  # -90° offset, quarter turn from offset
            (0.75, 0.25, 180),  # -90° offset, half turn from offset
            (0.75, 0.5, -90),  # -90° offset, three-quarter turn from offset
        ]

        for offset_degrees, cancoder_reading, expected_angle in test_cases:
            # Create module with the test offset
            test_module = SwerveModule("TestModule", 1, 2, 3, offset_rotations=offset_degrees)

            # Mock the cancoder reading
            def get_pos(r=cancoder_reading):
                class Position:
                    value = r

                return Position()

            test_module.can_coder.get_absolute_position = get_pos

            # Get the actual angle
            actual_angle = test_module.get_turn_angle_degrees()

            # Assert they match, with helpful error message
            self.assertAlmostEqual(
                expected_angle,
                actual_angle,
                places=2,
                msg=f"Failed with offset={offset_degrees}°, "
                    f"cancoder={cancoder_reading}, "
                    f"expected={expected_angle}°, "
                    f"got={actual_angle}°"
            )

class TestSwerveModuleOptimization(unittest.TestCase):
    def setUp(self):
        # Create instance of the class containing the optimize method
        self.swerve_module = SwerveModule("TestModule", 1, 2, 3, 0)  # Replace with your actual class name

    def test_small_angle_no_optimization(self):
        """Test when angle difference is small, no optimization needed"""
        current_rotation = 0.0  # 0 degrees
        desired_state = SwerveModuleState(1.0, Rotation2d.fromDegrees(45.0))

        result = self.swerve_module._optimize(desired_state, current_rotation)

        self.assertAlmostEqual(result.speed, 1.0)
        self.assertAlmostEqual(result.angle.degrees(), 45.0)

    def test_large_positive_angle_optimization(self):
        """Test when angle difference is > 90 degrees positive"""
        current_rotation = 0.0  # 0 degrees
        desired_state = SwerveModuleState(1.0, Rotation2d.fromDegrees(135.0))

        result = self.swerve_module._optimize(desired_state, current_rotation)

        # Should optimize to -45 degrees with negative speed
        self.assertAlmostEqual(result.speed, -1.0)
        self.assertAlmostEqual(result.angle.degrees(), -45.0)

    def test_large_negative_angle_optimization(self):
        """Test when angle difference is > 90 degrees negative"""
        current_rotation = 0.5  # 180 degrees
        desired_state = SwerveModuleState(1.0, Rotation2d.fromDegrees(0.0))

        result = self.swerve_module._optimize(desired_state, current_rotation)

        # Should optimize to 180 degrees with negative speed
        self.assertAlmostEqual(result.speed, -1.0)
        self.assertAlmostEqual(result.angle.degrees(), 180.0)

    def test_edge_case_90_degrees(self):
        """Test the edge case of exactly 90 degrees difference"""
        current_rotation = 0.0  # 0 degrees
        desired_state = SwerveModuleState(1.0, Rotation2d.fromDegrees(90.0))

        result = self.swerve_module._optimize(desired_state, current_rotation)

        # Should not optimize as it's exactly 90 degrees
        self.assertAlmostEqual(result.speed, 1.0)
        self.assertAlmostEqual(result.angle.degrees(), 90.0)

    def test_zero_speed(self):
        """Test optimization with zero speed"""
        current_rotation = 0.0
        desired_state = SwerveModuleState(0.0, Rotation2d.fromDegrees(180.0))

        result = self.swerve_module._optimize(desired_state, current_rotation)

        # Speed should remain zero, angle should optimize
        self.assertAlmostEqual(result.speed, 0.0)
        self.assertAlmostEqual(result.angle.degrees(), 0.0)


class TestTurnConversion(unittest.TestCase):
    def setUp(self):
        # Create instance of the class containing the degrees_to_turn_count method
        self.swerve_module = SwerveModule("TestSwerve", 1, 2, 3, 0)  # Replace with your actual class name

    def test_zero_degrees(self):
        """Test conversion of 0 degrees"""
        result = self.swerve_module._degrees_to_turn_count(0.0)
        self.assertAlmostEqual(result, 0.0)

    def test_full_rotation(self):
        """Test conversion of 360 degrees (full rotation)"""
        result = self.swerve_module._degrees_to_turn_count(360.0)
        # One full rotation * gear ratio
        self.assertAlmostEqual(result, DriveConstants.TURN_GEAR_RATIO)

    def test_half_rotation(self):
        """Test conversion of 180 degrees (half rotation)"""
        result = self.swerve_module._degrees_to_turn_count(180.0)
        # Half rotation * gear ratio
        self.assertAlmostEqual(result, DriveConstants.TURN_GEAR_RATIO / 2)

    def test_quarter_rotation(self):
        """Test conversion of 90 degrees (quarter rotation)"""
        result = self.swerve_module._degrees_to_turn_count(90.0)
        # Quarter rotation * gear ratio
        self.assertAlmostEqual(result, DriveConstants.TURN_GEAR_RATIO / 4)

    def test_negative_rotation(self):
        """Test conversion of -360 degrees (negative full rotation)"""
        result = self.swerve_module._degrees_to_turn_count(-360.0)
        # Negative one full rotation * gear ratio
        self.assertAlmostEqual(result, -DriveConstants.TURN_GEAR_RATIO)

    def test_multiple_rotations(self):
        """Test conversion of 720 degrees (two full rotations)"""
        result = self.swerve_module._degrees_to_turn_count(720.0)
        # Two full rotations * gear ratio
        self.assertAlmostEqual(result, 2 * DriveConstants.TURN_GEAR_RATIO)

    def test_arbitrary_angle(self):
        """Test conversion of an arbitrary angle (45 degrees)"""
        result = self.swerve_module._degrees_to_turn_count(45.0)
        # 45/360 rotation * gear ratio
        expected = (45.0 / 360.0) * DriveConstants.TURN_GEAR_RATIO
        self.assertAlmostEqual(result, expected)

if __name__ == '__main__':
    unittest.main()
