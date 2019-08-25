import cv2
import numpy as np
from ctypes import c_int, pointer

import darknet as dn


class DetectedTarget:
    '''
    Represents a target bounding box. Includes useful functions to retrieve
    bounding box properties on a given 2D image. Initialization of this object
    requires two arguments which describe the size of the image which is used
    to generate Darknet detections and the size of the image which the bounding box
    measurements should be in reference to.
    '''
    def __init__(self, detection_tuple, network_size, frame_size):
        '''
        Initializes a DetectedTarget.
        detection_tuple: A tuple of form (String, float, (float, float, float float))
            which can be received from Darknet. The detection tuple should contain
            information in the form (team color, confidence, (x, y, width, height)).
        network_size: A tuple (width, height) which denotes the frame size which was
            inputted into Darknet.
        frame_size: A tuple (width, height) which denotes the frame size that this
            DetectedTarget should have its measurements in reference to.
        '''
        (
            self.team_color,
            self.confidence,
            ( x, y, width, height)
        ) = detection_tuple

        # Unpack network size and frame size
        net_width, net_height = network_size
        frame_width, frame_height = frame_size

        # Convert to floats
        net_width = float(net_width)
        net_height = float(net_height)
        frame_width = float(frame_width)
        frame_height = float(frame_height)
        
        # Do conversions between the frame inputted into the neural net and the
        # size of the image the measurements of the bounding boxes should be in 
        # reference to
        self.x = x / net_width * frame_width
        self.y = y / net_height * frame_height
        self.width = width / net_width * frame_width
        self.height = height / net_height * frame_height
    
    def as_range_of(self, frame):
        '''
        Returns the section of an image this bounding box surrounds.

        frame: An image to cut the bounding box section out of.
        '''
        return frame[
            int(round(self.y - self.height / 2)):int(round(self.y + self.height / 2)), 
            int(round(self.x - self.width / 2)):int(round(self.x + self.width / 2)) 
        ]
    
    def center(self):
        '''
        Returns the center of this detected target in 2D.
        '''
        return (
            self.x,
            self.y
        )
    
    def top_left(self):
        '''
        Returns the top-left corner of this detected target's bounding box.
        '''
        return (self.x - self.width / 2, self.y - self.height / 2)
    
    def bottom_right(self):
        '''
        Returns the bottom-right corner of this detected target's bounding box.
        '''
        return (self.x + self.width / 2, self.y + self.height / 2)

PROD_CONFIG_FILE = "/home/nvidia/catkin_ws/src/aruw-vision-platform-2019/cv-prod-model/aruw-tiny-V2.cfg"
PROD_WEIGHTS_FILE = "/home/nvidia/catkin_ws/src/aruw-vision-platform-2019/cv-prod-model/aruw-tiny-V2_prod.weights"
PROD_USE_TEMPORAL = False

DATA_FILE = "/home/nvidia/catkin_ws/src/aruw-vision-platform-2019/cv-prod-model/aruw.data"
class DarknetDetector():
    '''
    Wraps Darknet and provides helpful functions for the ARUW implementation of Darknet.
    Can use temporal masks to increase the consistency of detections over time. Works by
    appending 4th and 5th channels to the image containing detections for both team colors
    for the past 20 frames scaled using a fibonacci sequence.
    '''
    def __init__(self, config_file=PROD_CONFIG_FILE, weights_file=PROD_WEIGHTS_FILE, use_temporal=PROD_USE_TEMPORAL):
        '''
        Initializes Darknet.
        config_file: File path to the Darknet config file which describes network parameters
        weights_file: File path to the Darknet weights file to load a trained network
        use_temporal: Whether or not to use the ARUW temporal consistency modifier which
            improves detections over time.
        '''
        self.net = dn.load_net(config_file, weights_file, 0)
        self.meta = dn.load_meta(DATA_FILE)
	self.net_size = (dn.network_width(self.net), dn.network_height(self.net))
        self.last_dets = []

        self.use_temporal = use_temporal

        self.PAST_FRAME_NUM = 20
        self.PAST_FRAME_PROPORTIONS = self.fibonacci_to_sum(255, self.PAST_FRAME_NUM)

    def fibonacci(self, n):
        '''
        Generate the n'th fibonacci number.
        '''
        if n == 1:
            return 1
        elif n == 2:
            return 2
        return self.fibonacci(n-1) + self.fibonacci(n-2)

    def fibonacci_to_sum(self, total, n):
        '''
        Generates a list of fibonacci numbers to n. Then scales that list
        so that the sum of that list sums to total.
        total: The sum to which the list of fibonacci numbers should total to.
        n: The length of the fibonacci number list.
        '''
        ints = [self.fibonacci(n) for n in range(1,n+1)]
        fib_total = sum(ints)
        return list(reversed([float(v)/fib_total*total for v in ints]))

    def make_dets_frame_mask(self, team_color, frame_shape):
        '''
        Makes temporal masks for a frame shape and team color
        team_color: The team color to make temporal masks for.
        frame_shape: The shape of the video frame 
        '''
        mask = np.zeros((frame_shape[0], frame_shape[1]), dtype="uint8")

        for frame,prop in reversed(list(zip(self.last_dets, self.PAST_FRAME_PROPORTIONS))):
            for target in frame:
                if not target.team_color == team_color:
                    continue
                subarr = target.as_range_of(mask)
                cv2.add(subarr, int(prop * target.confidence), subarr)
        
        return mask

    def clear_history(self):
        '''
        Clears the history of the temporal masks.
        '''
        self.last_dets = []

    def detect_targets(self, frame):
        '''
        Detects targets in a frame using Darknet. Returns a list of detections
        and appends detections to a history of detections
        frame: The 2D image to run detections on.
        '''
        frame_size = (frame.shape[1],frame.shape[0])
        frame = cv2.resize(frame, self.net_size)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # For use in debug. If use_temporal is "fake", appends black masks to the
        # 4th and 5th channels. Otherwise, if use_temporal is True, appends masks
        # from past frames to the image before passing it through the darknet detector.
        if self.use_temporal == "fake":
            augmented_frame = np.stack((
                frame[:,:,0],
                frame[:,:,1],
                frame[:,:,2],
                np.zeros((frame.shape[0], frame.shape[1]), dtype="uint8"),
                np.zeros((frame.shape[0], frame.shape[1]), dtype="uint8")
            ), axis=-1)
        elif self.use_temporal:
            augmented_frame = np.stack((
                frame[:,:,0],
                frame[:,:,1],
                frame[:,:,2],
                self.make_dets_frame_mask('red', frame.shape),
                self.make_dets_frame_mask('blue', frame.shape)
            ), axis=-1)
            
            #cv2.imshow("Red", augmented_frame[:,:,3])
            #cv2.imshow("Blue", augmented_frame[:,:,4])

        result = dn.detect_np(self.net, self.meta, augmented_frame if self.use_temporal else frame)
        self.last_dets.insert(0, [DetectedTarget(result_tuple, self.net_size, frame_size) for result_tuple in result])
        if len(self.last_dets) > self.PAST_FRAME_NUM:
            self.last_dets.pop()
        return self.last_dets[0]

    def _rounded_point(self, p):
        '''
        Helper function to round a 2D point to its nearest integer equivalent.
        p: The 2D point to round.
        '''
        return (int(round(p[0])), int(round(p[1])))

    def draw_detections(self, detections, frame):
        '''
        For debug. Draws detections on a frame. Returns a frame with bounding boxes
        drawn on it.
        detections: The detections outputted by Darknet
        frame: A 2D frame to draw bounding boxes on
        '''
        frame = frame.copy()
        for target in detections:
            color = { 'blue': (255, 128, 0), 'red': (0, 128, 255) }[target.team_color]

            cv2.rectangle(frame, self._rounded_point(target.top_left()), self._rounded_point(target.bottom_right()), color, 2)
            cv2.putText(frame, str(int(target.confidence * 100) / float(100)), self._rounded_point(target.top_left()), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255))
        return frame

if __name__ == "__main__":
    '''
    For debug.
    Runs Darknet Detector without any ROS dependencies. 
    '''
    import cv2

    demo_detectors = {
        #"Original tiny": DarknetDetector("/home/aruw/cv_data/aruw-tiny.cfg", "/home/aruw/cv_data/yolo_chkpts/aruw-tiny-1.1.0.weights", False),
        "New V2": DarknetDetector("/home/aruw/cv_data/aruw-tiny-V2.cfg", "/home/aruw/cv_data/yolo_chkpts/aruw-tiny-V2.backup", True),
        "New V2 without temporal": DarknetDetector("/home/aruw/cv_data/aruw-tiny-V2.cfg", "/home/aruw/cv_data/yolo_chkpts/aruw-tiny-V2.backup", "fake")
    }

    for detector_name in demo_detectors:
        cv2.namedWindow("Detections: " + detector_name, cv2.WINDOW_NORMAL)

    DEMO_VIDEO_SOURCE = "/home/aruw/cv_data/kaelin_test_video.avi"

    cap = cv2.VideoCapture(DEMO_VIDEO_SOURCE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame")
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        for detector_name, detector_obj in demo_detectors.iteritems():
            detections = detector_obj.detect_targets(frame)
            detections_visualization = detector_obj.draw_detections(detections, frame)

            cv2.imshow("Detections: " + detector_name, detections_visualization)

        cv2.waitKey(1)
