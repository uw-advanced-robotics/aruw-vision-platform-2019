import cv2 as cv
import numpy as np
import pyrealsense2 as rs
import math
from KCF import kcftracker
from utils import deproject_pixel_to_ros_point

'''
template_front = cv.imread('/home/nvidia/engy-align-data/binary_front_template.png', 0)
template_left = cv.imread('/home/nvidia/engy-align-data/binary_left_template.png', 0)
template_right = cv.imread('/home/nvidia/engy-align-data/binary_right_template.png', 0)
(tH, tW) = template_front.shape[:2]
'''

def distance(x, y, x1, y1):
    return math.sqrt((x-x1)**2 + (y-y1)**2)

def maximum(a, b, c):
    list = [a, b, c]
    return max(list), list.index(max(list))

# to make a new alignment target class, override the self.detect(frame_set) method

class Alignment(object):
    def __init__(self, camera_frame, target_frame):
        # State variables
        self.tracker_init = False
        self.detected = False
        self.tracker = kcftracker(False, False, True, False)  # hog, fixed_window, multiscale, LAB
        self.previous_coords = (0,0,0,0)
        self._latest_meas = None
        self._target_lost = False

        # RS variables
        self.camera_frame = camera_frame
        self.target_frame = target_frame

    def process_frame(self, frame_set):
        normalV = (0,0,1)
        (x,y,w,h) = (0, 0, 0, 0)
        #gray = cv.cvtColor(frame_set.color, cv.COLOR_BGR2GRAY)
        self._latest_meas = None

        if not self.detected:
            coords = self.detect(frame_set) #gray
            if coords is not None:
                (startX, startY, endX, endY) = coords
                '''
                # Check binarized image for white/black ratio
                target = gray[startY:endY,startX:endX]
                _, threshold = cv.threshold(target, 100, 255, cv.THRESH_BINARY)
                threshold = cv.erode(threshold, None, iterations=2)
                threshold = cv.dilate(threshold, None, iterations=2)
                whitePercentage = cv.countNonZero(threshold) / (threshold.size*1.0)
                if whitePercentage < 0.25 or whitePercentage > 0.90:
                    print("not target")
                    self.detected = False
                '''
                cv.rectangle(frame_set.color, (startX, startY), (endX, endY),(255,0,0),2)
                target = frame_set.color[startY:endY, startX:endX]
                #cv.imshow("target", target)
                
            
        if self.detected and not self.tracker_init:
            box = [startX, startY, endX-startX, endY-startY]
            self.tracker.init(box, frame_set.color)
            self.tracker_init = True
            self.previous_coords = box

        if self.tracker_init:
            box = self.tracker.update(frame_set.color)
            if box is not None:
                (x,y,w,h) = [int(v) for v in box]
                (x1,y1,w1,h1) = self.previous_coords
                #if distance(x, y, x1, y1) < 100 and abs(w1-w) < 50:    
                cv.rectangle(frame_set.color, (x,y), (x+w, y+h),(0,255,0),2)
                startX = x
                startY = y
                endX = x+w
                endY = y+h
                self._latest_meas = self.get_measurements(frame_set, (startX, startY, endX, endY))
                self.previous_coords = (x,y,w,h)
            else: #lost tracked target
                self.detected = False
                self.tracker_init = False
                self._target_lost = True

    '''
    # To be replaced with qr code
    def detect_plate(self, gray):
        found = None
        for scale in np.linspace(0.6, 1.0, 4)[::-1]:
            template_front_resized = cv.resize(template_front, None, fx=scale, fy=scale)
            template_left_resized = cv.resize(template_left, None, fx=scale, fy=scale)
            template_right_resized = cv.resize(template_right, None, fx=scale, fy=scale)
            w = template_front_resized.shape[0]
            h = template_front_resized.shape[1]
            r =  float(template_front_resized.shape[1])/template_front.shape[1]  
            edged = cv.Canny(gray, 50, 200)
            result_front = cv.matchTemplate(edged, template_front_resized, cv.TM_CCOEFF_NORMED)
            result_left = cv.matchTemplate(edged, template_left_resized, cv.TM_CCOEFF_NORMED)
            result_right = cv.matchTemplate(edged, template_right_resized, cv.TM_CCOEFF_NORMED)
            (_, maxVal1, _, maxLoc1) = cv.minMaxLoc(result_front)
            (_, maxVal2, _, maxLoc2) = cv.minMaxLoc(result_left)
            (_, maxVal3, _, maxLoc3) = cv.minMaxLoc(result_right)
            maxLocList = [maxLoc1, maxLoc2, maxLoc3]
            maxVal_left = maxVal2 * 2
            maxVal_right = maxVal3 * 2
            maxVal_front = maxVal1 * 2
            maxVal, index = maximum(maxVal_front, maxVal_left, maxVal_right)
            maxLoc = maxLocList[index]

            if found is None or maxVal > found[0]:
                found = (maxVal, maxLoc,r)
                
            (maxVal, maxLoc,r) = found

        (_, maxLoc, r) = found
        (startX, startY) = (int(maxLoc[0]), int(maxLoc[1]))
        (endX, endY) = (int((maxLoc[0] + tW * r)), int((maxLoc[1] + tH * r)))
        if maxVal >= 0.4:
            print maxVal
            self.detected = True
            return startX, startY, endX, endY
        print "not detected"
        return None
    '''

    def get_measurements(self, frame_set, (startX, startY, endX, endY)):
        cX = startgX + (endX - startX) / 2
        cY = startY + (endY - startY) / 2
        if cY >= frame_set.depth.shape[0] or cX >= frame_set.depth.shape[1]:
            print("Target out of frame")
            return self._latest_meas
        x, y, z = deproject_pixel_to_ros_point(cX, cY, frame_set.depth[cY, cX])
        a, b, c = self.get_normal_vector(frame_set, (startX, startY, endX, endY))
        normal_arctan = math.atan2(b, a)
        r = math.degrees(normal_arctan)
        return (x, y, r)

    def get_normal_vector(self, frame_set, (startX, startY, endX, endY)):
        depth_frame_16 = frame_set.depth
        df_dp = np.expand_dims(depth_frame_16, axis=-1).astype(np.uint8)
        df_dp = np.tile(df_dp, (1, 1, 3))
        depth_frame = depth_frame_16.astype(np.float32)
        cX = startX + (endX - startX) / 2
        cY = startY + (endY - startY) / 2
        sample = depth_frame[cY-20: cY+20, cX-20: cX+20]
        cv.rectangle(df_dp, (cX-20, cY-20), (cX+20, cY+20), (255,0,0), 2)
        dzdx = cv.Sobel(sample,cv.CV_32F,1,0,ksize=5)
        dzdy = cv.Sobel(sample,cv.CV_32F,0,1,ksize=5)
        dzdx = np.median(dzdx)
        dzdy = np.median(dzdy)
        #Convert to ros coordinates:
        # z->a, -dzdx->b, -dzdy->c
        return (1.0, -dzdx, -dzdy)

    def latest_meas(self):
        return self._latest_meas

    def lost_target(self):
        return self._target_lost

class SoldierTow(Alignment):
    def detect(self, frame_set):
        img = frame_set.color
        marker_centers, scale = findMarker(img)
        midpt = np.mean(marker_centers, axis=0)
        roi_side = math.sqrt(scale) * 3
        return (midpt[0]-roi_side, midpt[1]-roi_side,
                midpt[0]+roi_side, midpt[1]+roi_side)

def findMarker(img):
    # Returns:
    # centers: np array(2, 2) second axis - x, y
    # approx-scale: approximate scale of markers calculated
    # by area of largest marker detected
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #thresh = cv2.Canny(gray, 50, 200) #100, 200
    ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #ret, thresh = cv2.threshold(gray,200,255,cv2.THRESH_BINARY)
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    marker_cnts = []
    cv2.imshow("thresh", thresh)
    for i in range(len(contours)):
        trunc_contour = cv2.approxPolyDP(contours[i], 0.02*cv2.arcLength(contours[i], closed=True), closed=True)
        if (trunc_contour.size >=7 and trunc_contour.size <= 13):
            k = i
            h_count = 0
            # Find overarching parent of concentric contour
            while hierarchy[0, k, 2] != -1:
                k = hierarchy[0, k, 2]
                h_count += 1
            if h_count >= 2:
                marker_cnts.append(trunc_contour)
    #print(marker_cnts)
    cv2.drawContours(img, marker_cnts, -1, (0,255,0), 3)
    # each cnt is np array [1, n, 2] n is number of vertices
    n_codes = 2
    if len(marker_cnts) < 2:
        return np.empty((0, 2)).astype(int)
    marker_centers = np.empty((0, 2), dtype=np.float32)
    for cnt in marker_cnts:
        marker_center = np.mean(cnt, axis=0)
        marker_centers = np.concatenate((marker_centers, marker_center), axis=0)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    ret,label,centers = cv2.kmeans(marker_centers, n_codes, None, criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    centers = np.round(centers).astype(int)
    for center in centers:
        x = center[0]
        y = center[1]
        cv2.rectangle(img, (x-10, y-10), (x+10, y+10), (0, 0, 255))
    biggest = max(marker_cnts, key = cv2.contourArea)
    # second return value is approximate scale of marker detected
    return centers, cv2.contourArea(biggest)
