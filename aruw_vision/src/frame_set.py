class VisionFrameSet:
    '''
    Object that encapsulates all the frame data for a single
    aligned frame capture on the RealSense
    '''
    def __init__(self, color, depth, debug, ros_timestamp):
        '''
        Constructor
        @param color - the RGB frame
        @param depth - the depth frame
        @param debug - a copy of the RGB frame used for debug
                        drawing purposes
        @param ros_timestamp - the timestamp at which this frame was taken
        '''
        self.color = color
        self.depth = depth
        self.debug = debug
        self.ros_timestamp = ros_timestamp