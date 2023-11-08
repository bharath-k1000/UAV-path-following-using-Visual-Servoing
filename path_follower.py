#! /usr/bin/env python3
 
import rospy
import mavros
import mavros_msgs
import numpy as np
import os
import sys
import random
import cv2
 
import matplotlib.pyplot as plt
 
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Quaternion
from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOLRequest
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Imu
# importing pygame module
import pygame
# importing sys module
import sys
 
# angle for updating roll
ang = 0
 
# angle for updating pitch
ang2 = 0
 
# angle for going into/coming out of visual servoing mode
vs_var =0
 
current_state = State()
pose = PoseStamped()
twist = TwistStamped()
#current_pose =  NavSatFix()
 
 
alpha = 0.2
orientation = None
filtered_control = np.zeros((1, 2))
control= np.zeros((1, 2))
centroids = np.zeros((1, 2))
 
 
 
 
current_state = State()
pose = PoseStamped()
 
 
 
# Set up the visual servoing gains
kp = 0.08
ki = 0.001
kd = 0.08
# kp = 0.08
# ki = 0.0
# kd = 0.08
 
# kp = 0.1
# ki = 0.0
# kd = 0.0
# Initialize the integral and derivative errors
 
 
 
# Initialize the integral_error, derivative_error, and previous_error arrays
integral_error = np.zeros((1, 2))
derivative_error = np.zeros((1, 2))
previous_error = np.zeros((1, 2))
 
 
 
# Set up the camera intrinsics
fx = 277.191356
fy = 277.191356
cx = 320.5
cy = 240.5
 
intrinsics = np.array([[fx, 0.0, cx],
                       [0.0, fy, cy],
                       [0.0, 0.0, 1.0]])
 
# Set up the desired altitude
altitude = 1.0
 
# Set up the desired yaw rate
yaw_rate = 0.0
image_size_x = 320
image_size_y = 240
 
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 1
 
twist.twist.linear.x = 0.0
twist.twist.linear.y = 0.0
twist.twist.linear.z = 0.0
twist.twist.angular.x = 0.0
twist.twist.angular.y = 0.0
twist.twist.angular.z = 0.0
kp_yaw = 1.0 # Proportional gain for yaw control
yaw_degrees = 0
roll = 0
pitch = 0
orientation = None
 
 
def image_callback(data):
    global control, centroids, filtered_control
    bridge = CvBridge()
    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    image = cv_image
    cv2.imshow("image",image)
    
    # print(fit_curve_to_path(image))
    # thresh1 = fit_curve_to_path(image)
    # cv2.imshow("thresh1",thresh1)
    # thresh2 = segment_path(thresh1)
    # cv2.imshow("thresh2",thresh2)
    thresh = segment_path(image)
    cv2.imshow("thresh",thresh)
    centroids = find_centroid(thresh)
    # centroids2 = find_centroid(thresh2)
 
    output_frame = draw_centroid(image, [centroids[0]])
    # output_frame2 = draw_centroid(image, [centroids2[0]])
    # cv2.imshow("output_frame2",output_frame2)
    cv2.imshow("output_frame",output_frame)
    image_x = image.shape[1]
    image_y = image.shape[0]
    # print the dimensions of the image as a string
    # print("Image dimensions: " + str(image_x) + " x " + str(image_y))
    # control = visual_servoing_control(centroids,image_x, image_y, [kp, ki, kd])
    control = visual_servoing_control(centroids, image_x, image_y, [kp, ki, kd])
    filtered_control = alpha * control + (1 - alpha) * filtered_control
    
    cv2.waitKey(3)
 
# Callback function for the IMU data
def imu_callback(data):
    global orientation
    orientation = data.orientation
 
 
 
# Callback function for the local position data
def pose_callback(data):
    global pose_current
    pose_current = data.pose
 
 
def state_cb(msg):
    global current_state
    current_state = msg
 
 
 
def compute_visual_servoing_error(centroid, image_x, image_y):
 
    error = np.array([[(image_x / 2)-centroid[0][0], (image_y / 2)-centroid[0][1]]])
 
 
    return error
 
 
 
 
    # Define the visual_servoing_control function
def visual_servoing_control(centroid, image_x, image_y,  gains):
    global integral_error, derivative_error, previous_error
 
    # Compute the visual servoing error
    error = compute_visual_servoing_error(centroid, image_x, image_y)
 
    # Compute the control signal
    kp, ki, kd = gains
    #print(f"kp is {kp}, ki is {ki}, kd is {kd}")
 
    #u = -np.multiply(kp, error) - np.multiply(ki, integral_error) - np.multiply(kd, derivative_error)
    u = -kp * error- ki * integral_error - kd * derivative_error
 
    # Update the integral and derivative errors
    integral_error += error
    derivative_error = error - previous_error
    previous_error = error
    # print(f"x distance is {u[0][0]}, y distance is {u[0][1]}, z distance is {u[0][2]}")
    return u  # Return the control signal as a two-dimensional array
 
 
 
# def segment_path(frame):
#     # Segment out the path using a color threshold
#     lower_color = np.array([0, 20, 70])
#     upper_color = np.array([20, 255, 255])
#     mask = cv2.inRange(frame, lower_color, upper_color)
 
#     # Set upper 3/4ths of the image to black
#     height, width = mask.shape
#     mask[:-height//4*3, :] = 0
 
#     return mask
 
 
# def segment_path(frame):
#     # Convert the frame to HSV color space
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
#     # Define the range of skin color in HSV
#     lower_skin = np.array([0, 20, 70])
#     upper_skin = np.array([20, 255, 255])
 
#     # Threshold the frame to create a binary image of skin pixels
#     mask = cv2.inRange(hsv, lower_skin, upper_skin)
 
#     return mask
 
def segment_path(frame):
    # Set all pixels above the center line to black
   
    
    mask = np.zeros(frame.shape[:2], np.uint8)
    mask[150:240, 0:320] = 255
    
    frame = cv2.bitwise_and(frame, frame, mask=mask)
 
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    # Define the range of skin color in HSV
    lower_skin = np.array([0, 20, 70])
    upper_skin = np.array([20, 255, 255])
 
    # Threshold the frame to create a binary image of skin pixels
    mask = cv2.inRange(hsv, lower_skin, upper_skin)
 
    return mask
 
 
def fit_curve_to_path(frame):
    # Get the binary image of the path
    path_mask = segment_path(frame)
 
    # Find the contours of the path in the binary image
    contours, hierarchy = cv2.findContours(path_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
 
    # Fit a second-order polynomial curve to the path
    if len(contours) > 0:
        cnt = max(contours, key=cv2.contourArea)
        x, y = cnt[:, 0, 0], cnt[:, 0, 1]
        fit = np.polyfit(x, y, 2)
        curve = np.poly1d(fit)
        y_fit = curve(x)
 
        # Create a mask with only the points on the curve
        points_mask = np.zeros_like(path_mask)
        for i in range(len(x)):
            cv2.circle(points_mask, (int(x[i]), int(y_fit[i])), 1, (255, 0, 0), -1)
 
        # Apply the mask to the original image
        masked_image = cv2.bitwise_and(frame, frame, mask=points_mask)
 
        return masked_image
 
    else:
        return frame
 
 
 
def find_centroid(thresh):
    # Find the moments of the binary image
    moments = cv2.moments(thresh)
 
    # Extract the x and y coordinates of the centroid
    if moments['m00'] != 0:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
    else:
        # Set the centroid to None if the image is empty
        cx, cy = 0, 0
 
    #print(np.array([[cx, cy]]))
    return np.array([[cx, cy]])
 
def draw_centroid(image, centroids):
    # Make a copy of the original image to draw on
    output_image = image.copy()
    for centroid in centroids:
        cx, cy = centroid
        # Draw a circle at the centroid position
        cv2.circle(output_image, (cx, cy), 5, (0, 0, 255), -1)
    return output_image
 
 
 
   
 
def send_vel(centroids, u):  
        global pose, pose_current, ang, twist
    # Create the velocity command message
        
        quat = [orientation.w, orientation.x, orientation.y, orientation.z]
    # print(quat)
        euler = euler_from_quaternion(quat)
        yaw_rad_roll = euler[0]-(math.pi)/2
        yaw_rad_pitch = euler[0]
        if centroids is not None:
 
            # twist = TwistStamped()
            # twist.twist.linear.x = 4*(math.sin(yaw_rad_roll))
            # twist.twist.linear.y = 4*(math.cos(yaw_rad_roll))
            # twist.twist.angular.z = -u[0][0]
            # velocity_pub.publish(twist)
            # yaw_rad_roll+= -u[0][0]
            pose.pose.position.x += 0.04*(math.sin(yaw_rad_roll))
            pose.pose.position.y += 0.04*(math.cos(yaw_rad_roll))
            # # pose.pose.position.z =3.0
 
            # twist.twist.angular.z = -u[0][0]
 
            
 
            
            # new_angle =yaw_rad_roll
            # print(u[0][0])
            #print the current value of u[0][0]
            print(f"The value of u is {u[0][0]}")
            if (((u[0][0])>2) or ((u[0][0])<-2)):
                ang = yaw_rad_roll -0.1*u[0][0]
                #convert ang from radians to degrees
                ang = ang * 180 / math.pi
                temp = ang%360
                #convert temp from degrees to radians
                temp = temp * math.pi / 180
 
                
                quaternion = quaternion_from_euler(0, 0, temp)
                pose.pose.orientation = Quaternion(*quaternion)
 
            if (((u[0][0])>5) or ((u[0][0])<-5)):
                ang2 = yaw_rad_pitch-0.1*u[0][0]
                pose.pose.position.x += -0.008*(math.sin(ang2))
                pose.pose.position.y += -0.008*(math.cos(ang2))          
 
            # ang = yaw_rad_roll -0.1*u[0][0]
            # #convert ang from radians to degrees
            # ang = ang * 180 / math.pi
            # temp = ang%360
            # #convert temp from degrees to radians
            # temp = temp * math.pi / 180
 
            
            # quaternion = quaternion_from_euler(0, 0, temp)
            # pose.pose.orientation = Quaternion(*quaternion)
            
            
            local_pos_pub.publish(pose)
            # velocity_pub.publish(twist)
            # twist.twist.linear.z = -u[0][1]
            #twist.twist.angular.y = -u[0][1]
            # twist.twist.angular.x = -10.0
            
            
        else:
            # If the target is not found, stop the drone
            
            twist.twist.linear.x = 0.0
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0
            twist.twist.angular.x = 0.0
            twist.twist.angular.y = 0.0
            twist.twist.angular.z = 0.0
            
            pose.pose.position.x = pose_current.position.x
            pose.pose.position.y = pose_current.position.y
            pose.pose.position.z = pose_current.position.z
            pose.pose.orientation.z = pose_current.orientation.z
            pose.pose.orientation.w = pose_current.orientation.w
            pose.pose.orientation.x = pose_current.orientation.x
            velocity_pub.publish(twist)
            local_pos_pub.publish(pose)
 
 
def set_mode(mode):
    rospy.wait_for_service('mavros/set_mode')  # Waiting until the service starts 
    try:
        setmission = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode) # Creating a proxy service for the rosservice named /mavros/set_mode for arming the drone 
        setmission(0, mode)
        # print('2', setmission(220, "OFFBOARD"))
    except rospy.ServiceException as e:
        print ("Service set_mode call failed: %s"%e)
 
def setArm():
    # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
    rospy.wait_for_service('mavros/cmd/arming')  # Waiting until the service starts 
    try:
        armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
        armService(True)
    except rospy.ServiceException as e:
        print ("Service arming call failed: %s"%e)
def disArm():
    # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
    rospy.wait_for_service('mavros/cmd/arming')  # Waiting until the service starts 
    try:
        armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
        armService(False)
        rospy.loginfo("Vehicle disarmed")
    except rospy.ServiceException as e:
        print ("Service disarming call failed: %s"%e)
 
 
if __name__ == "__main__":
    
    rospy.init_node("offb_node_py")
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    imu_sub = rospy.Subscriber('mavros/imu/data', Imu, imu_callback)
    image_sub = rospy.Subscriber("iris/usb_cam/image_raw", Image, image_callback)
    pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_callback)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    velocity_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # rospy.wait_for_service("/mavros/set_mode")
    # takeoff_client = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
 
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)
    #rate_1 = rospy.Rate(50)
 
    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
 
 
    # initialising pygame
    pygame.init()
    
    # creating display
    display = pygame.display.set_mode((300, 300))
 
    while not rospy.is_shutdown():
        #pose = PoseStamped()
        #z = pose.pose.position.z
        if orientation is None:
            continue
 
    # Create the velocity command message
        
        twist.header.stamp = rospy.Time.now()
        quat = [orientation.w, orientation.x, orientation.y, orientation.z]
    # print(quat)
        euler = euler_from_quaternion(quat)
        yaw_rad_roll = euler[0]-(math.pi)/2
        yaw_rad_pitch = euler[0]
        # print("yaw - "+ str(yaw) + "deg")
        # yaw_rad = math.radians(yaw_degrees-120)
    # print(euler)
        local_pos_pub.publish(pose)
 
        if vs_var==1:
            # send_vel(centroids, filtered_control)
            send_vel(centroids, control)
            
 
 
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
        
        
                if event.key == pygame.K_w:
                    print("pitch - "+ str(pose.pose.position.y) + "m")
                    # twist.twist.linear.x = math.cos(yaw_rad)
                    # twist.twist.linear.y = math.sin(yaw_rad)
                    # velocity_pub.publish(twist)
                    pose.pose.position.y += 0.5*(math.cos(yaw_rad_roll))
                    pose.pose.position.x += 0.5*(math.sin(yaw_rad_roll))
                    
                elif event.key == pygame.K_s:
                    print("pitch - "+ str(pose.pose.position.y) + "m")
                    pose.pose.position.y -= 0.5*(math.cos(yaw_rad_roll))
                    pose.pose.position.x -= 0.5*(math.sin(yaw_rad_roll))
                elif event.key == pygame.K_d:
                    print("roll - "+ str(pose.pose.position.x) + "m")
                    pose.pose.position.y += 0.5*(math.cos(yaw_rad_pitch))
                    pose.pose.position.x += 0.5*(math.sin(yaw_rad_pitch))
                elif event.key == pygame.K_a:
                    print("roll - "+ str(pose.pose.position.x) + "m")
                    pose.pose.position.y -= 0.5*(math.cos(yaw_rad_pitch))
                    pose.pose.position.x -= 0.5*(math.sin(yaw_rad_pitch))
                elif event.key == pygame.K_z:
                    print("height - "+ str(pose.pose.position.z) + "m")
                    # z+=1
                    pose.pose.position.z+=0.2
 
                elif event.key == pygame.K_x:
                    print("height - "+ str(pose.pose.position.z) + "m")
                    pose.pose.position.z-=0.2  
 
                elif event.key == pygame.K_g:
                    # twist.twist.linear.x = 5.0
                    # twist.twist.linear.y = 5.0
                    # velocity_pub.publish(twist)
                    pose.pose.orientation.x+=1
                    
                elif event.key == pygame.K_h:
                    twist.twist.linear.x = 0.0
                    velocity_pub.publish(twist)
                    
 
 
                elif event.key == pygame.K_q:
                    # twist.twist.angular.z = 0.5
                    # velocity_pub.publish(twist)
                    yaw_degrees += 5
                    bla = yaw_degrees%360
                    yawk = math.radians(bla)
                    quaternion = quaternion_from_euler(0, 0, yawk)
                    pose.pose.orientation = Quaternion(*quaternion) 
                    # print("yawk - "+ str(yawk) + "\N{DEGREE SIGN}")
 
                    
                        
                elif event.key == pygame.K_e:
                    yaw_degrees -= 5
                    bla = yaw_degrees%360
                    yawk = math.radians(bla)
                    quaternion = quaternion_from_euler(0, 0, yawk)
                    pose.pose.orientation = Quaternion(*quaternion) 
                    # print("yawk - "+ str(yawk) + "\N{DEGREE SIGN}")
                    # twist.twist.angular.z = -0.5
                    # velocity_pub.publish(twist)
 
            
                
                elif event.key == pygame.K_l:
        
                    while current_state.mode != "AUTO.LAND":
                        set_mode("AUTO.LAND")
                        print (current_state.mode+  " activated")
                    disArm()
                    
                elif event.key == pygame.K_o:
                    # offb_set_mode.custom_mode = 'OFFBOARD'
                    # if(current_state.mode != "OFFBOARD"):
                    #     if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    #         rospy.loginfo( current_state.mode + " enabled")
                    set_mode("OFFBOARD")
                    rospy.loginfo( current_state.mode + " enabled")
                    arm_cmd = CommandBoolRequest()
                    arm_cmd.value = True
                    if(not current_state.armed):
                        if(arming_client.call(arm_cmd).success == True):
                                rospy.loginfo("Vehicle armed")
                
                elif event.key == pygame.K_n:
                    arm_cmd.value = True
                    if(not current_state.armed):
                        if(arming_client.call(arm_cmd).success == True):
                                rospy.loginfo("Vehicle armed")  
 
                elif event.key == pygame.K_m:
                    arm_cmd.value = False
                    if(current_state.armed):
                        if(arming_client.call(arm_cmd).success == True):
                                rospy.loginfo("Vehicle disarmed")   
 
                elif event.key == pygame.K_v:
                    vs_var=1
 
                elif event.key == pygame.K_b:
                    vs_var=0
 
                elif event.key == pygame.K_p:
                    pose.pose.position.x = 0
                    pose.pose.position.y = 0
                    pose.pose.position.z = 4
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 1
                    local_pos_pub.publish(pose)
 
 
 
    
        rate.sleep()
