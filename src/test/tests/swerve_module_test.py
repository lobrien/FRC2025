import unittest
from subsystems.swerve_module import SwerveModule

class SwerveModuleTest(unittest.TestCase):
    def test_get_turn_angle_degrees(self):
        test_module = SwerveModule("TestModule", 1, 2, 3, rotation_offset=0)
        self.assertIsNotNone(test_module)\

        angles_for_positions = {
            0 : 0,
            0.25 : 90,
            0.5 : 180,
            0.75 : -90,
        }
        for rotation, angle in angles_for_positions.items():
            # Mock the can_coder
            test_module._get_can_coder_pos_normalized = lambda: rotation
            angle_in_degrees = test_module.get_turn_angle_degrees()
            self.assertAlmostEqual(angle, angle_in_degrees)


if __name__ == '__main__':
    unittest.main()
