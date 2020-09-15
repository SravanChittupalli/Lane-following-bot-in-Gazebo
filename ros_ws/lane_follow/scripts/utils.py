
import cv2
import numpy as np
import math

kernel = np.ones((5 , 5) , np.float32)/25
lower_red = np.array([160,100,100])
upper_red = np.array([179,255,255])


def show_steering_corection(image , error):
    correction_image = np.zeros_like(image)
    error_radian = (error*math.pi)/180
    diff_in_x = 300*math.tan(error_radian)
    correct_x_coord = int(600-diff_in_x)
    cv2.line(correction_image , (600 , correction_image.shape[0]) , (correct_x_coord , correction_image.shape[0]-300) , (0 , 0 , 255) , 5)
    return correction_image



def compute_steering_angles(image , lane_lines):
    if len(lane_lines)==0:
        print("No lane lines detected")
        return -90
    
    height , width , _ = image.shape
    #print(lane_lines)
    if len(lane_lines) == 1:
        x1 , y1 , x2 , y2 = lane_lines[0].reshape(4)
        xoffset = x2 - x1
    else:
        lx1 , ly1 , lx2 , ly2 = lane_lines[0].reshape(4)
        rx1 , ry1 , rx2 , ry2 = lane_lines[1].reshape(4)
        mid = int(width/2)
        xoffset = int(((lx2 + rx2)/2) - mid)

    yoffset = int(height/2)
    if (xoffset) == 0:
        print(0)
        return 0
    angle_to_mid_radian = math.atan(yoffset/xoffset)
    angle_to_mid_degree = int(angle_to_mid_radian*180/math.pi)
    if(angle_to_mid_degree > 0 ):
        steering_angle = angle_to_mid_degree - 90
    else:
        steering_angle = angle_to_mid_degree + 90
    print(steering_angle)

    return steering_angle

def make_coordinates(image , line_parameteres):
    slope , intercept = line_parameteres
    width = image.shape[1]
    y1 = image.shape[0]
    y2 = int(y1*1/2)
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    #x1 = int((y1-intercept)/slope)
    #x2 = int((y2-intercept)/slope)
    return np.array([x1 , y1 , x2 , y2])



def apply_canny(cv_image):
    global kernal
    #ret,cv_image = cv2.threshold(cv_image,230,255,cv2.THRESH_BINARY)
    #cv2.imshow("img" , cv_image)
    #Here applying a 5X5 filter of ones is better than gaussian blur
    blured = cv2.filter2D(cv_image , -1 , kernel)
    canny = cv2.Canny(blured, 150, 190)
    return canny

def detect_red(cv_image):
    global lower_red , upper_red
    hsv_img = cv2.cvtColor(cv_image , cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img , lower_red , upper_red)
    red_edge = apply_canny(mask)
    return red_edge

def get_reqd_region(image):
    height = image.shape[0]
    polygon = np.array([[(0 , height) , 
                        (1200 , height), (1190 , 180 ),
                        (0 , 180)]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask , polygon , 255)
    cropped_img = cv2.bitwise_and(image , mask)
    return cropped_img


def  average_slope_intercept(image , lines):
    left_fit = []
    right_fit = []
    result = []

    boundary = 1/3
    left_region_boundary = image.shape[1] * (1 - boundary)
    right_region_boundary = image.shape[1] * (boundary)

    if lines is not None:
        for line in lines:
            x1 , y1 , x2 , y2 = line.reshape(4)
            if x1 == x2:
                print('Avoiding straight line')
                continue
            parameteres = np.polyfit((x1 , x2) , (y1 , y2) , 1) 
            slope = parameteres[0]
            intercept = parameteres[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope , intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope , intercept))
    if len(left_fit) != 0:
        left_line_params_avg = np.average(left_fit , axis=0)
        left_line = make_coordinates(image , left_line_params_avg )
        result.append(left_line)
    if len(right_fit) != 0:
        right_line_params_avg = np.average(right_fit , axis=0)
        right_line = make_coordinates(image , right_line_params_avg )
        result.append(right_line)
    return np.asarray(result)


def display_lines(image , lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1 , y1 , x2 , y2 = line.reshape(4)
            cv2.line(line_image , (x1 , y1) , (x2 , y2) , (255 , 0 , 0) , 10)
    return line_image
