import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time
import csv

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = True
        self.exceed_cnt = 0

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0
        msg = self.getModelState()
        pos_x, pos_y = msg.pose.position.x, msg.pose.position.y
        vel = np.sqrt(msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2)
        quaternion = msg.pose.orientation
        out = quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        yaw = out[2]
        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian
        
    def curvature_cal(self, x1, y1, x2, y2, x3, y3):
        k = abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
        s1 = np.linalg.norm([x1-x2, y1-y2], 2)
        s2 = np.linalg.norm([x2-x3, y2-y3], 2)
        s3 = np.linalg.norm([x1-x3, y1-y3], 2)
        k = 4000 * k /(s1 * s2 * s3)
        return k
    
    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        
        target_speed = 18  # default 10
        turn_vel = 10

        # Ensure there are at least two waypoints in the list to avoid IndexError
        # if len(future_unreached_waypoints) >= 2:
        #     x1, y1 = future_unreached_waypoints[0]
        #     x2, y2 = future_unreached_waypoints[1]
        # elif len(future_unreached_waypoints) == 1:
        #     # If there's only one waypoint left, use it for both x1,y1 and x2,y2
        #     x1, y1 = future_unreached_waypoints[0]
        #     x2, y2 = x1, y1  # Use the same waypoint as both the current and next target
        # else:
        #     # If there are no waypoints left, handle this case appropriately (e.g., stop the vehicle)
        #     return 0  # This line might need adjustment based on your application's needs

        # k1 = np.abs(curr_x * y1 - x1 * curr_y) / (curr_x**2 + curr_y**2)**(3/2)
        # k2 = np.abs(curr_x * y2 - x2 * curr_y) / (curr_x**2 + curr_y**2)**(3/2)
        # if (k1 < 1) or (k2 < 1):
        #     return target_speed
        # else:
        #     return turn_vel
        if len(future_unreached_waypoints) >= 2:
            x1, y1 = future_unreached_waypoints[0]
            x2, y2 = future_unreached_waypoints[1]
        elif len(future_unreached_waypoints) == 1:
            # If there's only one waypoint left, use it for both x1,y1 and x2,y2
            return target_speed
        else:
            # If there are no waypoints left, handle this case appropriately (e.g., stop the vehicle)
            return 0  # This line might need adjustment based on your application's needs

        k = self.curvature_cal(curr_x, curr_y, x1, y1, x2, y2)
        if k < 5:
            if np.abs(self.prev_vel - target_speed) > 5:
                return (target_speed + self.prev_vel) / 2
            return target_speed
        else:
            if np.abs(self.prev_vel - turn_vel) > 5:
                return (turn_vel + self.prev_vel) / 2
            return turn_vel

        # if len(future_unreached_waypoints)>1:
        #     x1, y1 = future_unreached_waypoints[0]
        #     x2, y2 = future_unreached_waypoints[1]
        # else:
        #     x1, y1 = future_unreached_waypoints[0]
        #     x2, y2 = future_unreached_waypoints[0]  
           
        #     print(" diff x1 is ",x1-curr_x , "diff y is", y1-curr_y)
        #     print(" diff x1 is ",x2-curr_x , "diff y is", y2-curr_y)
        # if ((((abs(x1-curr_x)<0.25) and abs(x2-curr_x)<0.25)) or ((abs(y1-curr_y)<0.25) and abs(y2-curr_y)<0.25)):
        #     target_velocity = 16  #11
        #     # print("straight")
        # else:
        #     target_velocity = 7  #8
        #     # print("curve")   

        # currentPose = 0
        # if(future_unreached_waypoints > 1):
        #     curvature = curvature_cal(cur_point, future_unreached_waypoints[0])
        # else:
        #     curvature = curvature_cal(cur_point, cur_point)
        
        # # Determine target speed based on curvature
        # if abs(curvature) < 0.001:  # Straight section 
        #     target_velocity = target_speed
        # else:
        #     target_velocity = turn_vel


     

        ####################### TODO: Your TASK 2 code ends Here #######################
        # return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        ##-----------------version wed 1.00pm
        wheel = self.L
        
        target_velocity = self.longititudal_controller(curr_x, curr_y, 0, curr_yaw, future_unreached_waypoints)
        # print(target_velocity)
        if target_velocity == 0 :
            return 0
        # print(target_velocity)
        i = 0 
        ld = 0
        cur_point = np.array([curr_x, curr_y])
        target = future_unreached_waypoints[0]
        ld =  np.sqrt(np.sum((future_unreached_waypoints[0] - cur_point)**2))

        alpha = np.arctan2((target[1]-cur_point[1]), (target[0]-cur_point[0])) - curr_yaw
        target_steering = np.arctan(2* wheel * np.sin(alpha)/ld)

        if(target_velocity > 15):
            ld_para = 7
            while(ld < ld_para and i < len(future_unreached_waypoints)):
                target = future_unreached_waypoints[i]
                ld =  np.sqrt(np.sum((target - cur_point)**2))
                i=i+1
                # print(ld)
                alpha = np.arctan2((target[1]-cur_point[1]), (target[0]-cur_point[0])) - curr_yaw
                target_steering = np.arctan(2* wheel * np.sin(alpha)/ld) 
                
        else: 
            target = future_unreached_waypoints[0]
            ld =  min (np.sqrt(np.sum((target - cur_point)**2)), 2)
            alpha = np.arctan2((target[1]-cur_point[1]), (target[0]-cur_point[0])) - curr_yaw
            target_steering = np.arctan(2* wheel * np.sin(alpha)/ld) 
              
        if(target_steering>1):
            target_steering = 1
        if(target_steering < -1):
            target_steering = -1
            

        return target_steering
        #----------------------------------another version wed 1.00pm
        
        # wheel = self.L

        # target_velocity = self.longititudal_controller(curr_x, curr_y, 0, curr_yaw, future_unreached_waypoints)
        # i = 0 
        # ld = 0
        # cur_point = np.array([curr_x, curr_y])
        # if future_unreached_waypoints:  # Check if the list is not empty
        #     target = future_unreached_waypoints[0]
        #     ld = np.sqrt(np.sum((target - cur_point)**2))
        #     alpha = np.arctan2((target[1]-cur_point[1]), (target[0]-cur_point[0])) - curr_yaw
        #     target_steering = np.arctan(2 * wheel * np.sin(alpha)/ld) 

        #     # Adjust target based on target_velocity and ensure i does not exceed list bounds
        #     if target_velocity == 12:
        #         ld_para = 7
        #         while ld < ld_para and i < len(future_unreached_waypoints) - 1:  # Prevent exceeding list bounds
        #             i += 1
        #             target = future_unreached_waypoints[i]
        #             ld = np.sqrt(np.sum((target - cur_point)**2))
        #             alpha = np.arctan2((target[1]-cur_point[1]), (target[0]-cur_point[0])) - curr_yaw
        #             target_steering = np.arctan(2 * wheel * np.sin(alpha)/ld)
        #     else:
        #         # If target_velocity is not 12, use the first waypoint or adjust as necessary
        #         ld = min(np.sqrt(np.sum((target - cur_point)**2)), 2)
        #         alpha = np.arctan2((target[1]-cur_point[1]), (target[0]-cur_point[0])) - curr_yaw
        #         target_steering = np.arctan(2 * wheel * np.sin(alpha)/ld)
        # else:
        #     # Handle the case where there are no future waypoints
        #     target_steering = 0  # Adjust as necessary for your application

        # # Clamp target_steering to a maximum/minimum value if needed
        # max_steering = np.sqrt(3)  # Example maximum steering angle
        # target_steering = max(min(target_steering, max_steering), -max_steering)

        # return target_steering

        
    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
            # print(acceleration)
        #     if acceleration > 5:
        #         self.exceed_cnt += 1
        #     with open("acceleration_log.csv", mode = "a", newline="") as file:
        #         writer = csv.writer(file)
        #         writer.writerow([acceleration])
                
            
        # with open("x_coor.csv", mode = "a", newline="") as file:
        #         writer = csv.writer(file)
        #         writer.writerow([curr_x])   
        # with open("y_coor.csv", mode = "a", newline="") as file:
        #         writer = csv.writer(file)
        #         writer.writerow([curr_y])   


        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)
        self.prev_vel = curr_vel

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
