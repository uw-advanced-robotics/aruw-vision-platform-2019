import math
import numpy as np
import pyrealsense2 as rs
import rospy
import cv2
import tf2_ros
from tracked_target import TrackedTarget

import utils
from common_utils import transform_to_frame, qv_mult, get_transform_prop

# TODO: base on covariance
TARGET_TRACKER_CORRELATION_DISTANCE_THRESHOLD = 1 # meters
TARGET_TRACKER_LOST_TIME_THRESHOLD = 0.5 # seconds

class TargetTracker:
    '''
    The Target Tracker tracks individual
    target plates across time to help in the identification
    of a primary target to shoot at.
    See our wiki's block diagram for more information
    '''
    def __init__(self, is_valid_target_predicate, target_rank_fn, camera_frame, static_frame, base_frame):
        '''
        Constructor
        @param is_valid_target_predicate - function reference to function that acceps a DetectedTarget as
                                            an argument and returns if it is a valid target
        @param target_rank_fn - function reference to function that accepts a TrackedTarget as an argument
                                and returns a key value for comparison and ranking
        @param camera_frame - TF frame which denotes the location of the RealSense RGB sensor
        @param static_frame - TF frame which is static to the world
        @param base_frame - TF frame which is static to the robot and centered at the turret axis
        '''
        self._camera_frame = camera_frame
        self._static_frame = static_frame 
        self._base_frame = base_frame

        self._is_valid_target_predicate = is_valid_target_predicate
        self._target_rank_fn = target_rank_fn

        self._tracked_targets = []

        self._is_tracking_primary_target = False
        self._primary_target = None

    def begin_tracking_primary_target(self):
        '''
        Select and designate a primary target
        '''
        self._is_tracking_primary_target = True
        self._pick_primary_target()
    
    def stop_tracking_primary_target(self):
        '''
        Remove the primary target
        '''
        self._is_tracking_primary_target = False
        self._primary_target = None

    def _init_new_target(self, stamped_static_point, team_color):
        '''
        Create a new target to be tracked in subsequent frames.
        @param stamped_static_point - PointStamped object containing position
                                        of the target plate inthe static frame
        @param team_color - team color of this target
        @return the newly created TrackedTarget object
        '''
        new_target = TrackedTarget(team_color, self._static_frame)
        new_target.update_and_correct(stamped_static_point)
        
        self._tracked_targets.append(new_target)
        return new_target
    
    def _get_ranked_valid_targets(self):
        '''
        Gets a list of valid tracked targets sorted in decreasing order
        of target priority
        @returns the sorted list
        '''
        valid_targets = filter(self._is_valid_target_predicate, self._tracked_targets)
        return sorted(valid_targets, key=self._target_rank_fn)
    
    def _pick_primary_target(self):
        '''
        Selects a primary target according to the _target_rank_fn.
        Sets _primary_target to None if there are no valid targets
        '''
        if not self._is_tracking_primary_target:
            self._primary_target = None
            return

        ranked_targets = self._get_ranked_valid_targets()
        if len(ranked_targets) == 0:
            self._primary_target = None
            return

        self._primary_target = ranked_targets[0]

    def _draw_vector_from_point(self, frame_set, base_point_stamped, vector_tuple, color):
        '''
        Draws an arrow specified in 3D space on the debug frame of a VisionFrameSet
        @param frame_set - VisionFrameSet whose debug frame is to be drawn on
        @param base_point_stamped - PointStamped object representing the coordinates of the point in the base frame
        @param vector_tuple - (x, y, z) of the arrow to be drawn
        @param color - color of the arrow to be drawn
        '''
        (base_x, base_y, base_z) = (base_point_stamped.point.x, base_point_stamped.point.y, base_point_stamped.point.z)
        (vector_x, vector_y, vector_z) = vector_tuple
        
        end_point = utils.point_stamped_with(base_point_stamped, (base_x + vector_x, base_y + vector_y, base_z + vector_z))

        base_pixel_coords = utils.deproject_ros_point_to_frame_point(base_point_stamped, frame_set, self._camera_frame)
        end_pixel_coords = utils.deproject_ros_point_to_frame_point(end_point, frame_set, self._camera_frame)

        cv2.arrowedLine(frame_set.debug, utils.rounded_point(base_pixel_coords), utils.rounded_point(end_pixel_coords), color, 3)

    def _draw_target_debug_info(self, frame_set):
        '''
        Draws target debug info on the debug frame of a VisionFrameSet.
        This includes a bounding rectangle for enemy plates, corner bracket
        rectangles for friendly plates, and vectors representing the standard
        deviation in position.
        @param frame_set - VisionFrameSet object whose debug frame is to be drawn on
        '''
        if frame_set.debug is None:
            return

        ranked_targets = self._get_ranked_valid_targets()
        for target in self._tracked_targets:
            is_valid_target = self._is_valid_target_predicate(target)

            color = { 'blue': (255, 128, 0), 'red': (0, 128, 255) }[target.team_color]
            thickness = 2 if target is not self._primary_target else 6

            if target.get_last_detect_target():
                _draw_rect = cv2.rectangle if is_valid_target else utils.draw_corner_bracket_rectangle
                _draw_rect(
                    frame_set.debug,
                    utils.rounded_point(target.get_last_detect_target().top_left()),
                    utils.rounded_point(target.get_last_detect_target().bottom_right()),
                    color,
                    thickness
                )

            if target.get_last_position():
                last_frame_point = utils.deproject_ros_point_to_frame_point(target.get_last_position(), frame_set, self._camera_frame)
                rounded_last_frame_point = utils.rounded_point(last_frame_point)
                cv2.circle(frame_set.debug, rounded_last_frame_point, 3, color, thickness, cv2.LINE_4)
                
                target_stddev = target.get_last_position_stddev()
                self._draw_vector_from_point(frame_set, target.get_last_position(), (target_stddev[0], 0, 0), (0, 0, 255))
                self._draw_vector_from_point(frame_set, target.get_last_position(), (0, target_stddev[1], 0), (0, 255, 0))
                self._draw_vector_from_point(frame_set, target.get_last_position(), (0, 0, target_stddev[2]), (255, 0, 0))

                if target in ranked_targets:
                    cv2.putText(
                        frame_set.debug,
                        str(ranked_targets.index(target)),
                        rounded_last_frame_point,
                        cv2.FONT_HERSHEY_DUPLEX,
                        1,
                        (0, 255, 255)
                    )


        if self._primary_target:
            origin_point = utils.deproject_ros_point_to_frame_point(
                utils.make_stamped_point((0.5, 0, 0), "base_turret", self._primary_target.get_last_position().header.stamp.to_sec()),
                frame_set,
                self._camera_frame
            )
            line_target_point = utils.deproject_ros_point_to_frame_point(
                utils.make_stamped_point((3, 0, 0), "base_turret", self._primary_target.get_last_position().header.stamp.to_sec()),
                frame_set,
                self._camera_frame
            )

            cv2.line(frame_set.debug, utils.rounded_point(origin_point), utils.rounded_point(line_target_point), (0, 255, 0), 1)

    def update_tracking(self, targets, frame_set, draw_targets=False):
        '''
        Updates the tracking kalman filters of currently tracked targets
        using the new detections of the latest frame received.
        See our wiki's page on Target Tracking and Target selection.
        @param targets - list of DetectedTarget objects that were detected in frame_set
        @param frame_set - the latest received frame from the camera
        @param draw_targets - boolean specifying whether to draw debug info for tracked targets
        @return latest position of the primary target if it exists, None otherwise
        '''
        existing_predictions = {
            target: target.predict_position_at_time(frame_set.ros_timestamp) for target in self._tracked_targets
        }

        for target_obj in targets:
            (target_x, target_y) = target_obj.center()
            roi = target_obj.as_range_of(frame_set.depth)
            detected_target_point = utils.deproject_target_rect_to_point(target_x, target_y, roi, frame_set.depth)

            if not detected_target_point:
                print("Rejecting frame")
                continue

            stamped_camera_relative_point = utils.make_stamped_point(detected_target_point, self._camera_frame, frame_set.ros_timestamp)
            stamped_static_point = transform_to_frame(stamped_camera_relative_point, self._static_frame)
            #print(stamped_static_point)
            
            if not stamped_static_point:
                continue
            
            def _dist_from_current(dist_target_obj):
                target_point = existing_predictions[dist_target_obj]
                return math.sqrt(
                    (target_point.point.x - stamped_static_point.point.x)**2
                    + (target_point.point.y - stamped_static_point.point.y)**2 
                    + (target_point.point.z - stamped_static_point.point.z)**2
                )

            available_targets = [t for t in existing_predictions.keys() if t.team_color == target_obj.team_color]
            sorted_targets = sorted(available_targets, key=_dist_from_current)
            closest = sorted_targets[0] if len(sorted_targets) > 0 else None

            if not closest or _dist_from_current(closest) > TARGET_TRACKER_CORRELATION_DISTANCE_THRESHOLD:
                # the closest possible correlated target is too far... we can't find a match
                # TODO: put this point on probation for a bit?
                new_target = self._init_new_target(stamped_static_point, target_obj.team_color)
                new_target.set_last_detect_target(target_obj)
                continue
            
            existing_predictions.pop(closest)

            closest.update_and_correct(stamped_static_point)
            closest.set_last_detect_target(target_obj)

            # for visualization
            #self.target_point_unfiltered_pub.publish(stamped_static_point)
            #self.target_point_filtered_pub.publish(smoothed_stamped_static_point)
            #self.target_point_pub.publish(predicted_static_point)
        
        # any targets remaining didn't have a pair
        must_pick_new_primary_target = self._primary_target == None
        for target in existing_predictions.keys():
            if frame_set.ros_timestamp - target.get_last_detected_timestamp() > TARGET_TRACKER_LOST_TIME_THRESHOLD:
                self._tracked_targets.remove(target)
                if self._primary_target == target:
                    must_pick_new_primary_target = True
            else:
                target.update_only_predict(frame_set.ros_timestamp)
                target.set_last_detect_target(None)
        
        if must_pick_new_primary_target:
            self._pick_primary_target()

        if draw_targets:
            self._draw_target_debug_info(frame_set)
        
        if self._primary_target is None:
            return None
        #print self._primary_target.get_kf_statePost()
        predicted_static_point = self._primary_target.get_last_position()
#            self._find_ballistics_intersection_point(self._primary_target, self._primary_target.get_last_updated_timestamp())

        #predicted_base_point = transform_to_frame(predicted_static_point, self._base_frame)
        #with open("/home/nvidia/tgt-log.csv", "a+") as f:
            #f.write(str(self._primary_target.get_last_detected_timestamp()) + "," + str(predicted_base_point.point.x) + "," + str(predicted_base_point.point.y) + "," + str(predicted_base_point.point.z) + "\n")


        return predicted_static_point

    def periodic_update(self, timestamp):
        '''
        For periodic updates in a separate thread.
        Not currently used in production
        '''
        for target in self._tracked_targets:
            target.update_only_predict(timestamp)

    def get_predicted_primary_target_state(self, timestamp):
        '''
        Predicts and returns the primary target state at a specified time
        @param timestamp - time to predict the primary target state to
        @return (x, y, z), (vx, vy, vz), (ax, ay, az)
                which is what the primary target's state is predicted to be
                at timestamp
        '''
        if self._primary_target is None:
            return None
        
        target_state = self._primary_target.predict_state_at_time(timestamp)
        # TODO: - time_offset of 0.03 might be shit
        predicted_position = utils.make_stamped_point(tuple(target_state[0:3]), self._static_frame, timestamp)
        predicted_velocity = utils.make_stamped_point(tuple(target_state[3:6]), self._static_frame, timestamp)
        predicted_acceleration = utils.make_stamped_point(tuple(target_state[6:9]), self._static_frame, timestamp)
        return (predicted_position, predicted_velocity, predicted_acceleration)

