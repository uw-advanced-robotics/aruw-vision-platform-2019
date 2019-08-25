import cv2
import numpy as np
from utils import point_stamped_with, make_stamped_point
from math import sqrt

class TrackedTarget:
    '''
    This class handles tracked target for the turret aiming system. 
    
    A specific single target (plate) is tracked, storing its position
    and velocity and updating them over time.
    '''
    
    def __init__(self, team_color, static_frame):
        '''
        Constructor
        @param team_color - team color
        @param static_frame - static TF frame that all measurements and
                                coordinate attributes of this object is in
        '''
        self._target_kf = None

        self.team_color = team_color

        self._static_frame = static_frame

        self._last_filtered_point = None
        self._last_filtered_velocity = None
        self._last_filtered_acceleration = None
        self._last_detect_target = None

        self._last_update_timestamp = None
        self._last_detected_timestamp = None
    
    def _init_kf(self, initial_position):
        '''
        Initializes the Kalman filter for tracking this plate
        @param initial_position - Initial position of the tracked plate
        '''
        self._target_kf = cv2.KalmanFilter(9, 3)

        # transitionMatrix will be edited before each predict/correct step with coefficients for the delta_t
        self._target_kf.transitionMatrix = np.eye(9, dtype=np.float32)
        self._target_kf.measurementMatrix = np.array([ # measurement_matrix * predicted_state = expected_measurement
            [1,   0,   0,   0,   0,   0,   0,   0,   0  ],
            [0,   1,   0,   0,   0  , 0,   0,   0,   0  ],
            [0,   0,   1,   0,   0,   0,   0,   0,   0  ],
        ], np.float32)

        # guessed covariances
        #self._target_kf.processNoiseCov = (0.1 **2) * np.eye(9, dtype=np.float32)
        self._target_kf.processNoiseCov = np.diag(np.array([
            0.05 **2,
            0.05 **2,
            0.05 **2,
            0.1 **2,
            0.1 **2,
            0.1 **2,
            0.01 **2,
            0.01 **2,
            0.01 **2
        ], dtype=np.float32))
        self._target_kf.measurementNoiseCov = (0.005 **2) * np.eye(3, dtype=np.float32)

        self._target_kf.statePre = np.zeros(9, dtype=np.float32)
        self._target_kf.statePre[0:3] = initial_position

        self._target_kf.statePost = np.reshape(np.copy(self._target_kf.statePre), (9,1))

    def _configure_kf_for_update(self, delta_t):
        '''
        Configures the state transition matrix of the Kalman filter
        according to the time delta
        in preparation for the predict step.
        @param delta_t - time delta, time elapsed between last measurement
                            and new measurement
        '''
        acc = 0.5 * delta_t**2 # acceleration factor (split out for matrix alignment)
        vel = 1 * delta_t # velocity/2nd-derivative factor
        np.fill_diagonal(self._target_kf.transitionMatrix[:6, 3:], vel)
        np.fill_diagonal(self._target_kf.transitionMatrix[:3, 6:], acc)

    def update_and_correct(self, target_point_stamped):
        '''
        Updates the Kalman filter by performing the
        predict and update steps using the newest measurement,
        and then updates the last filtered point.
        @param target_point_stamped - PointStamped object that represents the
                                        latest position measurement of this tracked plate
        @return the Kalman filter's new estimated position of this tracked plate
        '''
        current_timestamp = target_point_stamped.header.stamp.to_sec()
        
        current_pos = np.array([
            target_point_stamped.point.x,
            target_point_stamped.point.y,
            target_point_stamped.point.z
        ], np.float32)

        #with open("/home/nvidia/raw-log.csv", "a+") as f:
            #f.write(str(current_timestamp) + "," + str(current_pos[0]) + "," + str(current_pos[1]) + "," + str(current_pos[2]) + "\n")


        if not self._target_kf:
            self._init_kf(current_pos)

        self._configure_kf_for_update(current_timestamp - self._last_update_timestamp if self._last_update_timestamp else 0)
        self._target_kf.predict()
        filtered_measurement = self._target_kf.correct(current_pos)

        # Update state variables
        filtered_point = tuple(filtered_measurement[0:3,0])
        self._last_filtered_point = point_stamped_with(target_point_stamped, filtered_point)
        #print(self._target_kf.errorCovPost)

        self._last_update_timestamp = current_timestamp
        self._last_detected_timestamp = current_timestamp

        return self._last_filtered_point

    
    def update_only_predict(self, current_timestamp):
        '''
        Updates the last filtered point to the position predicted to
        the current time without performing the Kalman filter's update step.
        @param current_timestamp - timestamp of the current time to predict
                                    this tracked plate's position to
        @return the predicted position
        '''
        self._configure_kf_for_update(current_timestamp - self._last_update_timestamp if self._last_update_timestamp else 0)
        filtered_measurement = self._target_kf.predict()

        self._last_update_timestamp = current_timestamp

        filtered_point = tuple(filtered_measurement[0:3])
        self._last_filtered_point = point_stamped_with(self._last_filtered_point, filtered_point)
        return self._last_filtered_point

    def predict_state_at_time(self, predict_timestamp):
        '''
        Predicts the state of the tracked plate ahead to the specified timestamp
        @param predict_timestamp - timestamp of the time to predict
                                    this tracked plate's state to
        @return the predicted state
        '''
        # TODO: split out transition matrix gen so we don't update KF instance variables when we aren't using the KF
        time_delta = predict_timestamp - self._last_update_timestamp
        assert (time_delta >= 0)
        self._configure_kf_for_update(time_delta)
        return self._target_kf.transitionMatrix.dot(self._target_kf.statePost[:,0])

    def predict_position_at_time(self, predict_timestamp):
        '''
        Predicts the position of the tracked plate ahead to the specified timestamp
        @param predict_timestamp - timestamp of the time to predict
                                    this tracked plate's position to
        @return the predicted position as a PointStamp object in the static frame
        '''
        predicted_state = self.predict_state_at_time(predict_timestamp)
        return make_stamped_point(
            predicted_state[0:3],
            self._static_frame,
            predict_timestamp
        )
    
    
    def get_last_position_stddev(self):
        '''
        Get latest standard deviation of x, y, z position
        @return standard deviation of x, y, z. Tuple
        '''
        return (
            sqrt(self._target_kf.errorCovPost[0,0]),
            sqrt(self._target_kf.errorCovPost[1,1]),
            sqrt(self._target_kf.errorCovPost[2,2])
        )
    
    def set_last_detect_target(self, detect_target):
        '''
        Set last detected target as the given target
        @param detect_target The target to set as last detected
        '''
        self._last_detect_target = detect_target
    
    def get_last_detect_target(self):
        '''
        Finds last detected target
        @return detected target
        '''
        return self._last_detect_target

    def get_last_detected_timestamp(self):
        '''
        Finds timestamp of last target detection
        @return last detected timestamp
        '''
        return self._last_detected_timestamp
    
    def get_last_updated_timestamp(self):
        '''
        Finds last updated timestamp of tracked target
        @return last updated timestamp
        '''
        return self._last_update_timestamp
    
    def get_last_position(self):
        '''
        Finds lastest position of the tracked target
        @return last position
        '''
        return self._last_filtered_point

    def get_kf_statePost(self):
        '''
        Finds latest state of the tracked target's Kalman filter
        @return last state
        '''
        return self._target_kf.statePost

