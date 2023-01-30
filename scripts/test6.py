#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32 ,Bool
import os


width = 640
height = 360
count = 0

IMAGE_H = 360
IMAGE_W = 640

distance_x = 100
distance_y = 200

color=(255, 0, 0)
thickness=3

pre_true_ratio=0.0



stop_line = rospy.Publisher('detect_stop_line', Float32, queue_size=1)
stop_line_2 = rospy.Publisher('detect_stop_line_filtered', Float32, queue_size=1)
stop_line_3 = rospy.Publisher('is_stop_line', Bool, queue_size=1)


def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def canny(img, low_threshold, high_threshold):
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size, sigma):
    #return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), sigma)

def region_of_interest(img, vertices, color3=(255,255,255), color1=255):

    mask = np.zeros_like(img) # mask = img

    if len(img.shape) > 2:
        color = color3
    else:
        color = color1


    cv2.fillPoly(mask, vertices, color)


    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image

def draw_lines(img, lines, color=[0, 0, 255], thickness=4):
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

    return lines

def weighted_img(img, initial_img):
    return cv2.addWeighted(initial_img, 1, img, 1, 0)


def callback(data):
    global width, height
    global IMAGE_H, IMAGE_W, color ,thickness
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    src = np.float32([[0, 0], [IMAGE_W, 0],[0, IMAGE_H], [IMAGE_W, IMAGE_H]])
    dst = np.float32([[distance_x,distance_y], [IMAGE_W-distance_x,distance_y], [0, IMAGE_H], [IMAGE_W, IMAGE_H]])
    M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
    Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation
    #img = cv2.imread('./test_img.jpg')
    #image = image[450:(450+IMAGE_H), 0:IMAGE_W] # Apply np slicing for ROI crop
    warped_img = cv2.warpPerspective(image, Minv, (IMAGE_W, IMAGE_H)) # Image warping
    #plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB))
    #plt.show()

    #gray_img = grayscale(warped_img)
    blur_img = gaussian_blur(warped_img, 9, 3)
    #blur_img = gaussian_blur(gray_img, 3, 50)
    canny_img = canny(blur_img, 70, 140)

    #vertices = np.array([[(160, height-120), (160, 120), (width-160, 120), (width-160, height-120)]], dtype=np.int32)
    vertices = np.array([[(width/2, height), (width/2, height/2 + 70), (width, height/2 + 70), (width, height)]], dtype=np.int32)
    ROI_img = region_of_interest(canny_img, vertices)

    #hough_img = hough_lines(ROI_img, 1, 1*np.pi/180, 30, 10, 20)
    hough_img = hough_lines(ROI_img, 1, 1*np.pi/180, 30, 15, 25)

    #result = weighted_img(hough_img, image)
    #cv2.imshow('result4', canny_img)


    line_arr = hough_lines(ROI_img, 1, 1 * np.pi/180, 30, 15, 25)
    line_arr = np.squeeze(line_arr)
    temp = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

    #print(len(line_arr))
    true_ratio = float()
    true_false=Float32()


    if line_arr.size >4 :
        slope_degree = (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) *180 ) / np.pi

  #  line_arr = line_arr[np.abs(slope_degree)<160]
   # slope_degree = slope_degree[np.abs(slope_degree)<160]

        line_arr = line_arr[np.abs(slope_degree)>140]
        slope_degree = slope_degree[np.abs(slope_degree)>140]



        if line_arr.size >4:
            global count , pre_true_ratio ,true_ratio ,present_true_ratio
            #count = count+1

            #false_false=Float32()
            true_ratio = 1.0
            true_false.data = true_ratio
            stop_line.publish(true_false)

            L_lines, R_lines = line_arr[(slope_degree>0),:], line_arr[(slope_degree<0),:]
            temp = np.zeros((warped_img.shape[0], warped_img.shape[1], 3), dtype=np.uint8)
            L_lines, R_lines = L_lines[:,None], R_lines[:,None]

            draw_lines(temp, L_lines)
            draw_lines(temp, R_lines)

        else :
            true_ratio = 0.0
            true_false.data = true_ratio
            stop_line.publish(true_false)

        #present_true_ratio = pre_true_ratio*0.9 + true_ratio*0.1
        #pre_true_ratio = present_true_ratio

        #if present_true_ratio >= 0.7 :
         #   stop_line.publish(true_false)
        #else :
         #   stop_line.publish(true_false)


    else :
        true_ratio = 0.0
        true_false.data = true_ratio
        stop_line.publish(true_false)


    present_true_ratio = pre_true_ratio*0.8 + true_ratio*0.2
    pre_true_ratio = present_true_ratio
    present_true_ratio_msg = Float32()
    is_stop_line_msg = Bool()

    stop_line_2.publish(present_true_ratio)


    if present_true_ratio >= 0.7 :
        is_stop_line_msg.data = True
        stop_line_3.publish(is_stop_line_msg)


    else :
        is_stop_line_msg.data = False
        stop_line_3.publish(is_stop_line_msg)






   # L_lines, R_lines = line_arr[(slope_degree>0),:], line_arr[(slope_degree<0),:]
    #temp = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
    #L_lines, R_lines = L_lines[:,None], R_lines[:,None]



    result = weighted_img(temp, warped_img)
    result = cv2.rectangle(result, (width/2, height/2 + 70), (width, height), (255, 0, 0), 2)
    cv2.imshow('result',result), cv2.waitKey(1)
    #cv2.polylines(image, [dst], True, (255, 0, 255), 10)
    #cv2.line(image, (distance_x,distance_y), (IMAGE_W-distance_x,distance_y), color, thickness)
    #cv2.line(image, (IMAGE_W-distance_x,distance_y), (IMAGE_W, IMAGE_H), color, thickness)
    #cv2.line(image, (IMAGE_W, IMAGE_H), (0, IMAGE_H), color, thickness)
    #cv2.line(image, (0, IMAGE_H), (distance_x,distance_y), color, thickness)

    #cv2.imshow('image', image), cv2.waitKey(1)





if __name__ == "__main__" :
    rospy.init_node('stop_test')

    global distance_x, distance_y
    distance_x = 0 #int(input("distance_x (defalt 100) : "))
    distance_y = 0 #int(input("distance_y (defalt 200) : "))
    print("set distence_x, y are both 0 automatically")

    sub = rospy.Subscriber('kudos_cam_B/image_raw', Image, callback)

    #os.system("gnome-terminal -x rosbag play 2021-10-07-21-12-41.bag")
    #os.system("gnome-terminal -x rosbag play 2021-12-02-06-58-53.bag")

    #os.system("gnome-terminal -x rosbag play 2021-12-03-09-47-23.bag")

    #os.system("gnome-terminal -x rosbag play 2021-12-11-15-59-56.bag")
    #os.system("gnome-terminal -x rosbag play 2021-12-11-16-02-50.bag")



    rospy.spin()
