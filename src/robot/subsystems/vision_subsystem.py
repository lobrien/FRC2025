import math
from typing import Optional, Callable

import ntcore
import commands2
import logging

from wpimath.geometry import Pose2d
from wpimath.units import radiansToDegrees, seconds
# pip install limelightlib-python (0.9.6 for 2025)
import limelight
import limelightresults
from wpimath.estimator import SwerveDrive4PoseEstimator

from constants.new_types import inches, degrees, radians

logger = logging.getLogger(__name__)

class VisionSubsystem(commands2.Subsystem):
    def __init__(self, update_pose_estimate_fn : Callable) -> None:
        super().__init__()
        # self.desired_x_for_autonomous_driving = desired_auto_x
        # self.reef _x = reef _x

        self.networktables = ntcore.NetworkTableInstance.getDefault()
        self.limelight_table = self.networktables.getTable("limelight")
        self.botpose_subscription = self.limelight_table.getDoubleArrayTopic("botpose").subscribe([])
        self.botpose = [-1, -1, -1, -1, -1, -1]

        discovered_limelights = limelight.discover_limelights(debug=True)
        if len(discovered_limelights) == 0:
            logger.warning("No Limelight found!")
            self.limelight = None
        else:
            logger.info("Found Limelight!")
            self.limelight = limelight.Limelight(discovered_limelights[0])
            limelight_address = self.limelight.base_url
            logger.debug("Limelight address: %s", limelight_address)
            status = self.limelight.get_status()
            logger.info("Limelight status: %s", status)
            self.limelight.enable_websocket()
        self.limelight_result = None
        self.result_timestamp = None
        self.pose_estimator = pose_estimator

    def periodic(self) -> None:
        maybe_result = self._limelight_periodic()
        if maybe_result is not None:
            self._on_new_result(maybe_result)
        self._log_periodic()

    def debug_status(self) -> str:
        """
        Can be used for debugging purposes. Returns a string indicating whether the Limelight is found, if results are being received, and the time of the last result.
        """
        found_limelight = "Limelight found" if self.limelight is not None else "No Limelight found"
        getting_status = "Getting results..." if self.limelight_result is not None and self.limelight_result is not None else "No results available."
        time_status = f"Last result at {self.result_timestamp}" if self.result_timestamp is not None else "no results yet"
        return f"{found_limelight}, {getting_status}, {time_status}"

    def _limelight_periodic(self) -> Optional[limelightresults.GeneralResult]:
        if self.limelight is None:
            return None
        else:
            # Several results available. See https://docs.limelightvision.io/docs/docs-limelight/apis/limelightlib-python#websocket-based
            generalResult = self.limelight.get_results()
            if generalResult is None:
                return None
            else:
                # Per https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
                # TODO 2025-03-07: Confirm this index and put in VisionConsts
                megatag2_estimate = generalResult.botpose_wpiblue
                # Callback to drive subsystem with megatag2_estimate
                self.update_pose_estimator_fn(megatag2_estimate)
                self.limelight.update_robot_orientation()


                return generalResult

    def _log_periodic(self) -> None:
        # Temporary for diagnostics
        if self.limelight_result is not None:
            logger.debug("Limelight botpose: %s", self.limelight_result.botpose)
            logger.debug("Limelight result timestamp: %s", self.result_timestamp)
        else:
            logger.debug("No Limelight result")

    def _on_new_result(self, result: limelightresults.GeneralResult) -> None:
        self.limelight_result = result
        self.result_timestamp = result.timestamp

        # Callback to update odometry / pose-estimator
        robot_pose = Pose2d(result.botpose[0], result.botpose[1], radians(result.botpose[4]))
        self.add_vision_measurement_fn(robot_pose, self.result_timestamp)

    def get_botpose(self) -> Optional[list[float]]:
        if self.limelight_result is not None:
            return self.limelight_result.botpose
        else:
            return None
    
    @staticmethod
    def calculate_desired_x_distance(desired_x, bot_x):
        desired_pos = desired_x - bot_x
        return desired_pos
            

    @staticmethod
    def calculate_desired_direction(desired_angle, current_angle):
        """
        this function calculates the direction to travel for the robots yaw by saying if the value is greater than 180 it returns a negative number in degrees needed to travel
        if it is less it returns a positive amount of degrees to travel to get to the desired position.
        """
        desired_direction = desired_angle - current_angle
        if desired_direction > 180:
            desired_direction -= 360
        if desired_direction < -180:
            desired_direction += 360
        return desired_direction
    
    @staticmethod
    def distance_to_reef (bot_x, bot_y, reef_x, reef_y):
        """
        distance to reef  calculates the distance from the robots pos to the reef  in meters. it uses the distance formula subtracting the desired x,y (the reef )
        by our current x, y and square roots the answer to get our distance to be used in our shooter angle calculations. returns the distance from the robot to the reef
        """
        distance = math.sqrt(pow(reef_x - bot_x, 2) + pow(reef_y - bot_y, 2))
        return distance

    @staticmethod
    def calculate_desired_angle(distance_to_reef_y : inches, distance_to_wall : inches) -> degrees:
        """
        This function calculates the desired angle for the robot's orientation at different positions. It's measured by getting the distance to reef  on the y axis and the distance to the wall using the x axis
        and dividing them and uses the arctan function to calculate the angle needed to for the robot to rotate to into the reef  angle. This function returns the robots needed orientation in degrees.
        distance to reef Y is the adjacent angle
        distance to wall is the opposite angle
        """
        desired_angle_rad = radians(math.atan2(distance_to_reef_y, distance_to_wall))
        desired_angle = degrees(radiansToDegrees (desired_angle_rad))
        if desired_angle < 0:
            desired_angle += 360
        return desired_angle

    @staticmethod
    def get_rotation_autonomous_periodic_for_reef_shot(botpose, current_yaw):
        x = botpose[0]
        # desired_x = self.desired_x_for_autonomous_driving
        y = botpose[1]
       

        reef_y = 1.44 #
        # distance_to_wall = (self.reef _x - x) #Ajd
        # distance_to_reef _y = (reef _y - y) #Opp
        # # reef _distance = self.distance_to_reef (x, y, reef _x, reef _y) #Hy

        # desired_bot_angle = self.calculate_desired_angle(distance_to_reef _y, distance_to_wall)


        # direction_to_travel = self.calculate_desired_direction(desired_bot_angle, current_yaw)
        # direction_to_travel = self.radians_to_degrees(direction_to_travel_rad)
        # x_distance_to_travel = (desired_x - x)
        # x_kp = 0.007
        # x_max_speed = 0.2
        # # x_speed = x_kp * x_distance_to_travel

        # yaw_kp = 0.007
        # max_rot_value = 0.3
        # rot = yaw_kp * direction_to_travel
        # this acts like the p value in a pid loop for the rotation action

        # if x_speed > x_max_speed:
        #     x_speed = x_max_speed
        # elif x_speed < -x_max_speed:
        #     x_speed = -x_max_speed

        # if rot > max_rot_value:
        #     rot = max_rot_value
        # elif rot < -max_rot_value:
        #     rot = -max_rot_value
        #     # this sets makes sure that the rot value does not pass the maximum we give

        # return rot, direction_to_travel

# These are just for reference (they are defined in the limelightresults.py file)
# class GeneralResult:
#     def __init__(self, results):
#         self.barcode = results.get("Barcode", [])
#         self.classifierResults = [ClassifierResult(item) for item in results.get("Classifier", [])]
#         self.detectorResults = [DetectorResult(item) for item in results.get("Detector", [])]
#         self.fiducialResults = [FiducialResult(item) for item in results.get("Fiducial", [])]
#         self.retroResults = [RetroreflectiveResult(item) for item in results.get("Retro", [])]
#         self.botpose = results.get("botpose", [])
#         self.botpose_wpiblue = results.get("botpose_wpiblue", [])
#         self.botpose_wpired = results.get("botpose_wpired", [])
#         self.capture_latency = results.get("cl", 0)
#         self.pipeline_id = results.get("pID", 0)
#         self.robot_pose_target_space = results.get("t6c_rs", [])
#         self.targeting_latency = results.get("tl", 0)
#         self.timestamp = results.get("ts", 0)
#         self.validity = results.get("v", 0)
#         self.parse_latency = 0.0
#
# class RetroreflectiveResult:
#     def __init__(self, retro_data):
#         self.points = retro_data["pts"]
#         self.camera_pose_target_space = retro_data["t6c_ts"]
#         self.robot_pose_field_space = retro_data["t6r_fs"]
#         self.robot_pose_target_space = retro_data["t6r_ts"]
#         self.target_pose_camera_space = retro_data["t6t_cs"]
#         self.target_pose_robot_space = retro_data["t6t_rs"]
#         self.target_area = retro_data["ta"]
#         self.target_x_degrees = retro_data["tx"]
#         self.target_x_pixels = retro_data["txp"]
#         self.target_y_degrees = retro_data["ty"]
#         self.target_y_pixels = retro_data["typ"]
#
# class FiducialResult:
#     def __init__(self, fiducial_data):
#         self.fiducial_id = fiducial_data["fID"]
#         self.family = fiducial_data["fam"]
#         self.points = fiducial_data["pts"]
#         self.skew = fiducial_data["skew"]
#         self.camera_pose_target_space = fiducial_data["t6c_ts"]
#         self.robot_pose_field_space = fiducial_data["t6r_fs"]
#         self.robot_pose_target_space = fiducial_data["t6r_ts"]
#         self.target_pose_camera_space = fiducial_data["t6t_cs"]
#         self.target_pose_robot_space = fiducial_data["t6t_rs"]
#         self.target_area = fiducial_data["ta"]
#         self.target_x_degrees = fiducial_data["tx"]
#         self.target_x_pixels = fiducial_data["txp"]
#         self.target_y_degrees = fiducial_data["ty"]
#         self.target_y_pixels = fiducial_data["typ"]
#
# class DetectorResult:
#     def __init__(self, detector_data):
#         self.class_name = detector_data["class"]
#         self.class_id = detector_data["classID"]
#         self.confidence = detector_data["conf"]
#         self.points = detector_data["pts"]
#         self.target_area = detector_data["ta"]
#         self.target_x_degrees = detector_data["tx"]
#         self.target_x_pixels = detector_data["txp"]
#         self.target_y_degrees = detector_data["ty"]
#         self.target_y_pixels = detector_data["typ"]
#
# class ClassifierResult:
#     def __init__(self, classifier_data):
#         self.class_name = classifier_data["class"]
#         self.class_id = classifier_data["classID"]
#         self.confidence = classifier_data["conf"]
