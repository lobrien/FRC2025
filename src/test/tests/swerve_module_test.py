import unittest

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

from constants.driveconstants import DriveConstants
from subsystems.swerve_module import SwerveModule


class SwerveModuleTests(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.test_module = SwerveModule("TestModule", 1, 2, 3, offset_rotations=0)

    def _normalize_angle(self, rotation):
        """Convert rotation [0,1] to degrees in range (-180,180]"""
        degrees = rotation * 360.0
        # Normalize to (-180,180]
        degrees = (degrees + 180) % 360 - 180
        return degrees

    def test_basic_turn_angles(self):
        """Test basic turn angle calculations without offset."""
        angles_for_positions = {
            0: 0,  # 0 degrees
            0.25: 90,  # 0.25 rotations = 90 degrees
            0.5: 180,  # 0.5 rotations = 180 degrees
            0.75: -90,  # 0.75 rotations = 270 -> -90 degrees
            0.125: 45,  # 0.125 rotations = 45 degrees
            0.375: 135,  # 0.375 rotations = 135 degrees
            0.625: -135,  # 0.625 rotations = 225 -> -135 degrees
            0.875: -45,  # 0.875 rotations = 315 -> -45 degrees
        }
        for rotation, expected_angle in angles_for_positions.items():
            with self.subTest(rotation=rotation, expected_angle=expected_angle):
                self.test_module._get_can_coder_pos_normalized = lambda: rotation
                actual_angle = self.test_module.get_turn_angle_degrees()
                self.assertAlmostEqual(
                    expected_angle,
                    actual_angle,
                    places=3,
                    msg=f"Failed at rotation={rotation}, expected={expected_angle}°, got={actual_angle}°"
                )

    def test_turn_angles_with_offset(self):
        """Test turn angle calculations with various rotational offsets."""
        test_cases = [
            # (offset_rotations, cancoder_reading, expected_angle)
            # For each test case:
            # true_angle = cancoder_reading * 360
            # normalized_angle = normalize(true_angle) to (-180,180]

            # No offset (offset = 0.0)
            (0.0, 0.0, 0),  # 0.0 * 360 = 0°
            (0.0, 0.25, 90),  # 0.25 * 360 = 90°
            (0.0, 0.5, 180),  # 0.5 * 360 = 180°
            (0.0, 0.75, -90),  # 0.75 * 360 = 270° -> -90°

            # 90° offset (offset = 0.25)
            (0.25, 0.0, 0),  # 0.0 * 360 = 0°
            (0.25, 0.25, 90),  # 0.25 * 360 = 90°
            (0.25, 0.5, 180),  # 0.5 * 360 = 180°
            (0.25, 0.75, -90),  # 0.75 * 360 = 270° -> -90°

            # 180° offset (offset = 0.5)
            (0.5, 0.0, 0),  # 0.0 * 360 = 0°
            (0.5, 0.25, 90),  # 0.25 * 360 = 90°
            (0.5, 0.5, 180),  # 0.5 * 360 = 180°
            (0.5, 0.75, -90),  # 0.75 * 360 = 270° -> -90°

            # 270° offset (offset = 0.75)
            (0.75, 0.0, 0),  # 0.0 * 360 = 0°
            (0.75, 0.25, 90),  # 0.25 * 360 = 90°
            (0.75, 0.5, 180),  # 0.5 * 360 = 180°
            (0.75, 0.75, -90),  # 0.75 * 360 = 270° -> -90°
        ]

        for offset, reading, expected in test_cases:
            with self.subTest(offset=offset, reading=reading, expected=expected):
                # Create new module with test offset
                test_module = SwerveModule("TestModule", 1, 2, 3, offset_rotations=offset)
                test_module._get_can_coder_pos_normalized = lambda: reading
                actual = test_module.get_turn_angle_degrees()

                self.assertAlmostEqual(
                    expected,
                    actual,
                    places=2,
                    msg=f"Failed with offset={offset}, reading={reading}, expected={expected}°, got={actual}°"
                )

    def test_optimization_cases(self):
        """Test various cases for swerve module state optimization."""
        test_cases = [
            # (current_rotation, desired_speed, desired_angle, expected_speed, expected_angle)
            # Small angle case
            (0.0, 1.0, 45.0, 1.0, 45.0),
            # Minimal angle difference
            (0.0, 1.0, 1.0, 1.0, 1.0),
            # Large positive angle case
            (0.0, 1.0, 135.0, -1.0, -45.0),
            # Large negative angle case
            (0.5, 1.0, 0.0, -1.0, 180.0),
            # Edge case at 90 degrees
            (0.0, 1.0, 90.0, 1.0, 90.0),
            # Edge case at -90 degrees
            (0.0, 1.0, -90.0, 1.0, -90.0),
            # Zero speed case
            (0.0, 0.0, 180.0, 0.0, 0.0),
            # Near 180 boundary case
            (0.0, 1.0, 179.0, -1.0, -1.0)
        ]

        for current_rot, des_speed, des_angle, exp_speed, exp_angle in test_cases:
            with self.subTest(current_rotation=current_rot, desired_angle=des_angle):
                desired_state = SwerveModuleState(des_speed, Rotation2d.fromDegrees(des_angle))
                result = self.test_module._optimize(desired_state, Rotation2d.fromRotations(current_rot))

                self.assertAlmostEqual(
                    result.speed,
                    exp_speed,
                    msg=f"Speed mismatch: expected {exp_speed}, got {result.speed}"
                )
                self.assertAlmostEqual(
                    result.angle.degrees(),
                    exp_angle,
                    msg=f"Angle mismatch: expected {exp_angle}, got {result.angle.degrees()}"
                )

    def test_turn_count_conversion(self):
        """Test conversion between degrees and turn counts."""
        test_cases = [
            (0.0, 0.0),  # Zero degrees
            (360.0, DriveConstants.TURN_GEAR_RATIO),  # Full rotation
            (180.0, DriveConstants.TURN_GEAR_RATIO / 2),  # Half rotation
            (90.0, DriveConstants.TURN_GEAR_RATIO / 4),  # Quarter rotation
            (-360.0, -DriveConstants.TURN_GEAR_RATIO),  # Negative full rotation
            (720.0, 2 * DriveConstants.TURN_GEAR_RATIO),  # Multiple rotations
            (45.0, (45.0 / 360.0) * DriveConstants.TURN_GEAR_RATIO),  # Arbitrary angle
            (-45.0, (-45.0 / 360.0) * DriveConstants.TURN_GEAR_RATIO),  # Negative arbitrary angle
            (0.5, (0.5 / 360.0) * DriveConstants.TURN_GEAR_RATIO),  # Small angle
        ]

        for degrees, expected_count in test_cases:
            with self.subTest(degrees=degrees):
                result = self.test_module._degrees_to_turn_count(degrees)
                self.assertAlmostEqual(
                    result,
                    expected_count,
                    msg=f"Failed converting {degrees}° to turn count. Expected {expected_count}, got {result}"
                )

    def test_place_in_appropriate_scope_basic(self):
     """Test basic cases for placing angles in appropriate scope."""
     test_cases = [
         # (reference, angle, expected)
         (0, 0, 0),        # Same angle
         (0, 90, 90),      # Basic positive
         (0, -90, -90),    # Basic negative
         (0, 180, 180),    # Edge case
         (0, -180, 180),  # Negative edge case

         # Reference at 90
         (90, 0, 0),
         (90, 180, 180),
         (90, 270, 270),

         # Reference at 180
         (180, 0, 0),
         (180, 90, 90),
         (180, 270, 270),
     ]
     for ref, angle, expected in test_cases:
         with self.subTest(reference=ref, angle=angle):
             result = self.test_module._place_in_appropriate0_to360_scope(ref, angle)
             self.assertAlmostEqual(expected, result)

    def test_place_in_appropriate_scope_wrapping(self):
     """Test wrapping behavior for placing angles in scope."""
     test_cases = [
         # (reference, angle, expected)
         # Wrapping with reference at 0
         (0, 361, 1),       # Just over one rotation
         (0, 721, 1),       # Just over two rotations
         (0, -361, -1),     # Just over negative one rotation
         (0, -721, -1),     # Just over negative two rotations

         # Wrapping with reference at 180
         (180, 540, 180),   # 540 = 180   360
         (180, -180, 180),  # Staying at 180 is ok
         (180, 900, 180),   # Multiple rotations

         # Reference at arbitrary angle (45)
         (45, 405, 45),     # One rotation   reference
         (45, -315, 45),    # Negative wrap to reference
     ]
     for ref, angle, expected in test_cases:
         with self.subTest(reference=ref, angle=angle):
             result = self.test_module._place_in_appropriate0_to360_scope(ref, angle)
             self.assertAlmostEqual(expected, result)

    def test_place_in_appropriate_scope_180_boundary(self):
     """Test behavior near the 180-degree boundary."""
     test_cases = [
         # (reference, angle, expected)
         # Reference at 0
         (0, 179, 179),      # Just under 180
         (0, -179, -179),    # Just under -180
         (0, -181, 179),     # Just under -180, should wrap
         (0, 180, 180),      # Exactly 180 should stay positive

         # Reference at 90
         (90, 269, 269),     # 179 degrees from reference
         (90, 271, -89),     # 181 degrees from reference, should wrap
         (90, 270, 270),     # Exactly 180 from reference

         # Reference at 180
         (180, 359, 359),    # 179 degrees from reference
         (180, 361, 1),      # 181 degrees from reference, should wrap
         (180, 0, 0)
     ]
     for ref, angle, expected in test_cases:
         with self.subTest(reference=ref, angle=angle):
             result = self.test_module._place_in_appropriate0_to360_scope(ref, angle)
             self.assertAlmostEqual(
                 expected,
                 result,
                 msg=f"Failed with reference={ref}°, angle={angle}°. Expected {expected}°, got {result}°"
             )

if __name__ == '__main__':
    unittest.main()