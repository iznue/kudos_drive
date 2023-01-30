#!/usr/bin/env python
# coding: utf-8
# 완성된 코드에 정지선 인식 토픽 받아오기

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from std_msgs.msg import Int8
from std_msgs.msg import UInt16

width = 640
height = 360
pub = None
pub_state_ = None
angle = 0.0
stop_line = None
count_stop = 0.0
count_false = 0.0
Mode = 5 #None
count_real = 0.0
final_count = 1

src = np.float32([[300, 360], [0, 360], [0, 210], [300, 210],])
dst = np.float32([[250, 400], [50, 400], [35, -30], [245, -30], ])


def pub_state():
    global pub_state_
    msg_ = UInt16()
    msg_.data = 1
    pub_state_.publish(msg_)


def drive(speed, angle):
    global pub
    twist = Twist()
    twist.linear.x = speed
    twist.angular.z = angle

    pub.publish(twist)

def image_process(img):
    kernel_size = 9
    blur_img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 3)

    low_threshold = 70
    high_threshold = 140
    canny_img = cv2.Canny(blur_img, low_threshold, high_threshold)

    kernel1 = np.ones((20, 20), np.uint8)
    close1_img = cv2.morphologyEx(canny_img, cv2.MORPH_CLOSE, kernel1)

    kernel2 = np.ones((3, 3), np.uint8)
    close2_img = cv2.erode(close1_img, kernel2, iterations = 1)

    W = cv2.getPerspectiveTransform(src, dst)
    warp_img = cv2.warpPerspective(close2_img, W, (width, height), flags=cv2.INTER_LINEAR)

    ret, threshold_img = cv2.threshold(warp_img, 150, 255, cv2.THRESH_BINARY)

    return threshold_img

def unwarp(img):
    W = cv2.getPerspectiveTransform(dst, src)
    image = cv2.warpPerspective(img, W,(width, height), flags=cv2.INTER_LINEAR)
    return image

def region_of_interest(img):
    mask = np.zeros_like(img)

    if len(img.shape) > 2:
        color = (255, 255, 255)
    else:
        color = 255
    vertices = np.array([[(0, 0), (0, 360), (200, 360), (200, 0)]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, color)
    roi_img = cv2.bitwise_and(img, mask)
    return roi_img

def hough_lines(img):
    rho = 1
    theta = 1*np.pi/180
    threshold = 30
    min_line_len = 10
    max_line_gap = 20
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), min_line_len, max_line_gap)

    return lines

def draw_lines(img, lines):
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

def weighted_img(img, initial_img):
    return cv2.addWeighted(initial_img, 1, img, 1, 0)

def get_fitline(img, f_lines):

    lines = np.squeeze(f_lines)
    lines = lines.reshape(lines.shape[0] * 2, 2)
    rows, cols = img.shape[:2]
    output = cv2.fitLine(lines,cv2.DIST_L2,0, 0.01, 0.01)
    vx, vy, x, y = output[0], output[1], output[2], output[3]
    x1, y1 = int(((img.shape[0]-1)-y)/vy*vx + x) , img.shape[0]-1
    x2, y2 = int(((img.shape[0]/2+100)-y)/vy*vx + x) , int(img.shape[0]/2+100)

    result = [x1, y1, x2, y2]
    return result

def draw_fit_line(img, lines, color=[0, 0, 255], thickness=10):
    cv2.line(img, (lines[0], lines[1]), (lines[2], lines[3]), color, thickness)

def set_angle(loc_line):
    if loc_line <= 70:
        angle = 0.15
    elif (loc_line > 70) and (loc_line <= 130):
        angle = 0.0
    else:
        angle = -0.15

    return angle

def modeCallback(msg):
    global Mode
    Mode = msg.data

def callback_stop(msg):
    global stop_line, count_stop, count_false
    stop_line = msg.data

    if stop_line == True:
        count_stop = count_stop + 1
    else:
        count_false = count_false + 1
        if count_false > 30:
            count_stop = 0.0
            #print "initialize"

def callback(data):
    global stop_line, count_stop, count_false, angle, count_real, final_count
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    pro_img = image_process(frame)
    roi_img = region_of_interest(pro_img)

    lines = hough_lines(roi_img)

    if (Mode == 5) and (final_count == 1):
        if not lines is None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 - x1 == 0:
                    slope = 0.0
                else:
                    slope = float(y2 - y1) / float(x2 - x1)
            line_img = np.zeros((frame.shape[0], frame.shape[1], 3), dtype = np.uint8)

            fit_line = get_fitline(frame, lines)
            draw_fit_line(line_img, fit_line)

            line_img = unwarp(line_img)
            result = weighted_img(line_img, frame)
            cv2.imshow('result', result), cv2.waitKey(1)

            loc_line = x1 + (x2 - x1) / 2
            angle = set_angle(loc_line)
            #print stop_line

            if count_stop == 2:
                count_real = count_real + 1
                if count_real == 3:
                    i = 0
                    while (i < 70):
                        drive(0.07, 0.01)
                        rospy.sleep(0.1)
                        i = i + 1
                        print "while"
                    final_count = final_count + 1

                print "stop"
                rospy.sleep(4)
                count_stop = count_stop + 10

                print count_real

            else:
                speed = 0.07
                drive(speed, angle)

        speed = 0.07
        drive(speed, angle)
    else:
        drive(0, 0)

    pub_state()

if __name__ == "__main__":
    rospy.init_node('test')
    rate = rospy.Rate(1)

    sub = rospy.Subscriber('/kudos_cam_B/image_raw', Image, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    sub_mode = rospy.Subscriber('/mode', Int8, modeCallback)

    stop_line_3 = rospy.Subscriber('is_stop_line', Bool, callback_stop)

    pub_state_ = rospy.Publisher('autodriving_state', UInt16, queue_size = 10)

    rospy.spin()
