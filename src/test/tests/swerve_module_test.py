import unittest
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

if __name__ == '__main__':
    unittest.main()
