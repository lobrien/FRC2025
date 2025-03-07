import unittest
from unittest.mock import MagicMock, patch
import math

# Import the VisionSubsystem class and related dependencies
import ntcore
import commands2
from subsystems.vision_subsystem import VisionSubsystem
import limelightresults


class TestVisionSubsystem(unittest.TestCase):

    def setUp(self):
        # with patch('ntcore.NetworkTableInstance') as mock_nt_instance, \
        #         patch('limelight.discover_limelights') as mock_discover_limelights:
        #     mock_nt = MagicMock()
        #     mock_table = MagicMock()
        #     mock_subscription = MagicMock()
        #
        #     # Configure mocks for network tables
        #     mock_nt_instance.getDefault.return_value = mock_nt
        #     mock_nt.getTable.return_value = mock_table
        #     mock_table.getDoubleArrayTopic().subscribe.return_value = mock_subscription
        #
        #     # Mock limelight discovery
        #     mock_discover_limelights.return_value = ["test_limelight_address"]

            # Create the VisionSubsystem
            self.vision_subsystem = VisionSubsystem()

            # Mock the limelight object
            self.vision_subsystem.limelight = MagicMock()

    def test_init(self):
        """Test initialization of VisionSubsystem."""
        self.assertIsNotNone(self.vision_subsystem)
        self.assertEqual(self.vision_subsystem.botpose, [-1, -1, -1, -1, -1, -1])
        self.assertIsNone(self.vision_subsystem.limelight_result)
        self.assertIsNone(self.vision_subsystem.result_timestamp)

    def test_debug_status_with_no_results(self):
        """Test debug_status method when no results are available."""
        status = self.vision_subsystem.debug_status()
        self.assertIn("Limelight found", status)
        self.assertIn("No results available", status)
        self.assertIn("no results yet", status)

    def test_debug_status_with_results(self):
        """Test debug_status method when results are available."""
        # Mock a result
        mock_result = MagicMock()
        mock_result.timestamp = 12345.0

        self.vision_subsystem.limelight_result = mock_result
        self.vision_subsystem.result_timestamp = mock_result.timestamp

        status = self.vision_subsystem.debug_status()
        self.assertIn("Limelight found", status)
        self.assertIn("Getting results", status)
        self.assertIn("Last result at 12345.0", status)

    def test_limelight_periodic_no_limelight(self):
        """Test _limelight_periodic when no limelight is available."""
        self.vision_subsystem.limelight = None
        result = self.vision_subsystem._limelight_periodic()
        self.assertIsNone(result)

    def test_limelight_periodic_with_results(self):
        """Test _limelight_periodic when results are available."""
        mock_result = MagicMock()
        self.vision_subsystem.limelight.get_results.return_value = mock_result

        result = self.vision_subsystem._limelight_periodic()
        self.assertEqual(result, mock_result)

    def test_limelight_periodic_no_results(self):
        """Test _limelight_periodic when no results are available."""
        self.vision_subsystem.limelight.get_results.return_value = None

        result = self.vision_subsystem._limelight_periodic()
        self.assertIsNone(result)

    def test_on_new_result(self):
        """Test _on_new_result method."""
        mock_result = MagicMock()
        mock_result.timestamp = 12345.0

        self.vision_subsystem._on_new_result(mock_result)

        self.assertEqual(self.vision_subsystem.limelight_result, mock_result)
        self.assertEqual(self.vision_subsystem.result_timestamp, 12345.0)

    def test_checkBotpose_with_no_result(self):
        """Test checkBotpose when no result is available."""
        self.vision_subsystem.limelight_result = None
        botpose = self.vision_subsystem.get_botpose()
        self.assertIsNone(botpose)

    def test_checkBotpose_with_result(self):
        """Test checkBotpose when a result is available."""
        mock_result = MagicMock()
        mock_result.botpose = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

        self.vision_subsystem.limelight_result = mock_result

        botpose = self.vision_subsystem.get_botpose()
        self.assertEqual(botpose, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_calculate_desired_x_distance(self):
        """Test calculate_desired_x_distance method."""
        desired_x = 10.0
        bot_x = 7.0

        result = self.vision_subsystem.calculate_desired_x_distance(desired_x, bot_x)
        self.assertEqual(result, 3.0)

    def test_calculate_desired_direction_less_than_180(self):
        """Test calculate_desired_direction when difference is less than 180."""
        desired_angle = 90.0
        current_angle = 45.0

        result = self.vision_subsystem.calculate_desired_direction(desired_angle, current_angle)
        self.assertEqual(result, 45.0)

    def test_calculate_desired_direction_greater_than_180(self):
        """Test calculate_desired_direction when difference is greater than 180."""
        desired_angle = 350.0
        current_angle = 10.0

        result = self.vision_subsystem.calculate_desired_direction(desired_angle, current_angle)
        self.assertEqual(result, -20.0)

    def test_calculate_desired_direction_less_than_negative_180(self):
        """Test calculate_desired_direction when difference is less than -180."""
        desired_angle = 10.0
        current_angle = 350.0

        result = self.vision_subsystem.calculate_desired_direction(desired_angle, current_angle)
        self.assertEqual(result, 20.0)

    def test_distance_to_reef(self):
        """Test distance_to_reef method."""

        bot_x = 1.0
        bot_y = 2.0
        reef_x = 4.0
        reef_y = 6.0

        result = self.vision_subsystem.distance_to_reef(bot_x, bot_y, reef_x, reef_y)

        # The expected result is 5.0 (Pythagorean theorem: √((4-1)² + (6-2)²) = √(9 + 16) = √25 = 5)
        self.assertAlmostEqual(result, 5.0, places=1)

    def test_calculate_desired_angle(self):
        """Test calculate_desired_angle method."""
        distance_to_reef_y = 3.0  # Adjacent
        distance_to_wall = 4.0  # Opposite

        # Expected angle is atan2(y, x) in radians, converted to degrees
        expected_angle = math.degrees(math.atan2(distance_to_reef_y, distance_to_wall))

        result = self.vision_subsystem.calculate_desired_angle(distance_to_reef_y, distance_to_wall)

        self.assertAlmostEqual(result, expected_angle, places=1)

    def test_calculate_desired_angle_negative(self):
        """Test calculate_desired_angle method with negative result."""
        distance_to_reef_y = -3.0  # Adjacent
        distance_to_wall = 4.0  # Opposite

        # Expected angle is atan2(y, x) in radians, converted to degrees, then adjusted if negative
        expected_angle = math.degrees(math.atan2(distance_to_reef_y, distance_to_wall))
        if expected_angle < 0:
            expected_angle += 360

        result = self.vision_subsystem.calculate_desired_angle(distance_to_reef_y, distance_to_wall)

        self.assertAlmostEqual(result, expected_angle, places=1)

    def test_periodic_with_no_result(self):
        """Test periodic method when no result is available."""
        with patch.object(self.vision_subsystem, '_limelight_periodic', return_value=None), \
                patch.object(self.vision_subsystem, '_log_periodic') as mock_log_periodic:
            self.vision_subsystem.periodic()

            mock_log_periodic.assert_called_once()

    def test_periodic_with_result(self):
        """Test periodic method when a result is available."""
        mock_result = MagicMock()

        with patch.object(self.vision_subsystem, '_limelight_periodic', return_value=mock_result), \
                patch.object(self.vision_subsystem, '_on_new_result') as mock_on_new_result, \
                patch.object(self.vision_subsystem, '_log_periodic') as mock_log_periodic:
            self.vision_subsystem.periodic()

            mock_on_new_result.assert_called_once_with(mock_result)
            mock_log_periodic.assert_called_once()


if __name__ == '__main__':
    unittest.main()