import math

import numpy as np
import rospy
from tf2_geometry_msgs import PointStamped
import pyrealsense2 as rs
import cv2

from common_utils import transform_to_frame

def make_3ch_scaled_depth(depth):
    '''
    Makes a 3-channel image from a depth frame
    with different colors representing different depths
    for visualization purposes.
    @param depth - the depth frame to make the image from
    @return the created 3-channel image
    '''
    result = np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)
    result[:,:,0] = depth / (6000 / 255)
    result[:,:,1] = depth / (6000 / 255)
    result[:,:,2] = depth / (6000 / 255)
    return result

def point_stamped_with(original_point, point_tuple):
    '''
    Creates a new PointStamped object with the original point's
    header data (frame and timestamp) but with new xyz data.
    @param original_point - the original point to copy the 
                                header data from
    @param point_tuple - the new xyz data. Tuple (x, y, z)
    @return the created PointStamped object
    '''
    new_point = PointStamped()
    new_point.header.frame_id = original_point.header.frame_id
    new_point.header.stamp = original_point.header.stamp
    [
        new_point.point.x,
        new_point.point.y,
        new_point.point.z
    ] = point_tuple

    return new_point

def make_stamped_point(point_tuple, ros_frame, ros_timestamp):
    '''
    Creates a new PointStamped object using the specified data.
    @param point_tuple - xyz data. Tuple (x, y, z)
    @param ros_frame - the TF frame that the point is in
    @param ros_timestamp - the ROS timestamp that the point was measured
    @return the created PointStamped object
    '''
    new_point = PointStamped()
    new_point.header.frame_id = ros_frame
    new_point.header.stamp = rospy.Time.from_sec(ros_timestamp)
    [
        new_point.point.x,
        new_point.point.y,
        new_point.point.z
    ] = point_tuple

    return new_point

class CameraInfo(object):
    '''
    This class contains the color intrinsic
    parameters of the RealSense cameras
    for use by the RealSense APIs
    '''
    def __init__(self):
        self._color_intrinsics= None
    def set_color_intrinsics(self, color_intrinsics):
        self._color_intrinsics = color_intrinsics
    def get_color_intrinsics(self):
        return self._color_intrinsics

cameraInfo = CameraInfo()

def deproject_pixel_to_ros_point(frame_x, frame_y, depth):
    '''
    Converts a pixel from its 2-D value to the real camera-relative 3-D 
    measurements of the object that it captures.
    @param frame_x - x-coordinate of pixel in frame
    @param frame_y - y-coordinate of pixel in frame
    @param depth - depth reading of the pixel in the aligned depth frame
    @return the 3-D point in ROS coordinates: +x forward, +y left, +z up
    '''
    global cameraInfo
    rs2_point = rs.rs2_deproject_pixel_to_point(cameraInfo.get_color_intrinsics(), [frame_x, frame_y], depth)
    return (
        # convert from right-handed z-forward to right-handed z-up
        rs2_point[2] / 1000, # z
        -rs2_point[0] / 1000, # -x
        -rs2_point[1] / 1000 # -y
    )

def deproject_ros_point_to_frame_point(target_point, frame_set, camera_frame):
    '''
    Converts a 3-D camera-relative point to a 2-D pixel coordinate in frame
    @param target_point - ROS point that is being converted.
    @param frame_set - the latest VisionFrameSet object
    @param camera_frame - the TF frame that represents the camera's frame of reference
    @return the 3-D point in ROS coordinates: +x forward, +y left, +z up
    '''
    global cameraInfo
    target_point_cam_rel = transform_to_frame(target_point, camera_frame)
    if not target_point_cam_rel:
        return None
    rs2_point = [
        # convert from right-handed z-up to right-handed z-forward
        -target_point_cam_rel.point.y * 1000,
        -target_point_cam_rel.point.z * 1000,
        target_point_cam_rel.point.x * 1000,
    ]
    return tuple(rs.rs2_project_point_to_pixel(cameraInfo.get_color_intrinsics(), rs2_point))

def rounded_point(p):
    '''
    Rounds a 2-D image point to the closest integer value
    since pixel coordinates of image arrays are integers only.
    @param p - the 2-D point to be rounded. List-like (x, y)
    @return the rounded point. Tuple (x, y)
    '''
    return (int(round(p[0])), int(round(p[1])))

def deproject_target_rect_to_point(cX, cY, roi, depth_frame):
    '''
    Converts a rectangular area on a frame
    from its 2-D values to the real camera-relative 3-D 
    measurements of the object that it captures.
    @param cX - x-coordinate of center pixel of rectangle in frame
    @param cY - y-coordinate of center pixel of rectangle in frame
    @param roi - the rectangular subarray of the frame to be deprojected
    @param depth_frame - the depth frame
    @return the 3-D point in ROS coordinates: +x forward, +y left, +z up
    '''
    target_depth = np.nanmedian(roi)
    
    if (math.isnan(target_depth)
            or roi.size < 30
            or float(np.count_nonzero(roi == np.nan) + np.count_nonzero(roi == 0)) / roi.size > 0.2):
        # Error handling by caller
        return None

    return deproject_pixel_to_ros_point(cX, cY, target_depth)

def point_add(a, b):
    '''
    Adds 2 points
    @param a - first point. List-like
    @param b - second point. List-like
    @return result of a + b. Tuple
    '''
    return tuple(np.add(a, b))

def point_sub(a, b):
    '''
    Subtracts b from a
    @param a - first point. List-like
    @param b - point to be subtracted. List-like
    @return result of a - b. Tuple
    '''
    return tuple(np.subtract(a, b))

def point_dist(a, b):
    '''
    Calculates euclidean distance between 2 points.
    @param a - first point. List-like
    @param b - second point. List-like
    @return the calculated euclidean distance.
    '''
    assert (len(a) == len(b) and (len(a) in (1, 2, 3)))
    return math.sqrt(np.sum(np.subtract(a, b) **2))

def point_mul(point, a):
    '''
    Multiplies a point by a scalar
    @param point - point to be multiplied. List-like
    @param a - scalar factor
    @return result of a*point. Tuple
    '''
    return tuple(np.array(point)*a)

def draw_corner_bracket_rectangle(frame, top_left, bottom_right, color, thickness):
    '''
    Draws a stylized, four-corner-bracketed, rectangle on a frame.
    @param frame - frame to draw on
    @param top_left - pixel coordinates of the top_left corner. Tuple (x, y)
    @param bottom_right - pixel coordinates of the bottom_right color. Tuple (x, y)
    @param color - color of the rectangle to be drawn
    @param thickness - line thickness of the rectangle to be drawn
    '''
    (left, top) = top_left
    (right, bottom) = bottom_right
    top_right = (right, top)
    bottom_left = (left, bottom)

    width = abs(right - left)
    height = abs(top - bottom)

    line_len = min(width, height) * 0.2

    all_corner_pairs = [
        (top_left, top_right),
        (top_left, bottom_left),
        (bottom_right, top_right),
        (bottom_right, bottom_left)
    ]

    # for each pair of corners on the same side...
    for (corner_a, corner_b) in all_corner_pairs:
        # for each permutation of that pair...
        for (start, point_toward) in ((corner_a, corner_b), (corner_b, corner_a)):
            # draw a line from the first one in the direction of the other
            # but only part of the distance
            side_direction_vector = point_mul(
                point_sub(point_toward, start),
                1/point_dist(start, point_toward)
            )

            cv2.line(
                frame,
                rounded_point(start),
                rounded_point(point_add(start, point_mul(side_direction_vector, line_len))),
                color,
                thickness
            )
