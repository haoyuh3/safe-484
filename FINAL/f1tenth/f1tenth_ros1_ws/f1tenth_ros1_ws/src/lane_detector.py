import time
import math
import numpy as np
import cv2
import rospy
import os
import pathlib


from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


import argparse

parser = argparse.ArgumentParser()

# lane detector arguments
parser.add_argument('--output_dir', '-o', type=str, default='')
parser.add_argument('--output_freq', type=int, default=1,
                    help='output frame to folder. Requires output_dir not empty.')
parser.add_argument('--sat_cdf_lower_thres', type=float,
                    default=0.5, help='heuristic lower thres for saturation cdf.')
parser.add_argument('--blue_red_diff_thres', type=int,
                    default=30, help='blue and red channel difference thres')
parser.add_argument('--val_thres_percentile', type=int,
                    default=65, help='value percentile thres')
parser.add_argument('--red_val_tolerance', type=int, default=15)
parser.add_argument('--hue_thresh', type=str, default='15,40',
                    help='valid range for hue channel after HSL conversion')
parser.add_argument('--dilate_size', type=int, default=5,
                    help='kernel size for closing technique to remove noise')
parser.add_argument('--perspective_pts', '-p',
                    type=str, default='115,525,294,0', help='param for perpective projection. Four number represents respectively for src_leftx, src_rightx, laney, offsety.')
parser.add_argument('--window_height', type=int, default=20,
                    help='window height for lane fitting.')

# controller arguments
parser.add_argument('--steering_k', type=float, default=0.35)
parser.add_argument('--steering_i', type=float, default=1.8)
parser.add_argument('--curv_min', type=float, default=0.0)
parser.add_argument('--curv_max', type=float, default=1.5)
parser.add_argument('--angle_limit', type=float, default=30)
parser.add_argument('--vel_min', type=float, default=0.5)
parser.add_argument('--vel_max', type=float, default=0.5)
parser.add_argument('--look_ahead', type=float, default=-1.0,
                    help='fixed look ahead distance for pure pursuit controller. -1 denotes dynamic lookahead distance, which directly use the most distant waypoint as target point')
parser.add_argument('--max_look_ahead', type=float,
                    default=0.4, help='In case car looks too far away.')
parser.add_argument('--obstacle_tolerate_dist', type=float, default=-np.inf,
                    help='car change velocity if obstacle is within this distance. Negative infinity means the car does not avoid obstacle')

class LaneDetector():
    def __init__(self, args, debug_mode=False):
        self.parse_params(args)
        self.debug_mode = debug_mode
        self.output_dir = args.output_dir
        self.output_freq = args.output_freq
        
        # controller params
        self.steering_k = args.steering_k
        self.steering_i = args.steering_i
        self.angle_limit = args.angle_limit
        self.curv_min = args.curv_min
        self.curv_max = args.curv_max
        self.vel_min = args.vel_min
        self.vel_max = args.vel_max
        self.look_ahead = args.look_ahead
        self.max_look_ahead = args.max_look_ahead
        self.obstacle_tolerate_dist = args.obstacle_tolerate_dist
        self.wheelbase = 0.325
        self.debug_mode = debug_mode
        self.way_pts = []
        self.last_vel = 0
        self.last_steering = 0
        self.reach_boundary = True
        self.cnt = 0


        #obstacle avoidance 

        self.obstacle_avoidance_MUX = True
        self.obstacle_planning_withoutpts = False
        self.obstacle_start_time = 0
        self.obstacle_detected = False
        # self.obstacle_max = 0
        # self.obstacle_min = 0
        self.obstacle_MUX = False

        if not self.debug_mode:
            self.bridge = CvBridge()
            self.sub_image = rospy.Subscriber('/camera/color/image_raw', Image, self.img_callback, queue_size=1)
            self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)
            
            # self.sub_image = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size=1)
            self.ctrl_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
            self.drive_msg = AckermannDriveStamped()
            self.drive_msg.header.frame_id = "f1tenth_control"


    def parse_params(self, args):
        # parse params
 

        self.hue_thres_min, self.hue_thres_max = args.hue_thresh.split(',')
        self.hue_thres_min, self.hue_thres_max = int(self.hue_thres_min), int(self.hue_thres_max)
        assert self.hue_thres_min < self.hue_thres_max

        src_leftx, src_rightx, laney, offsety = args.perspective_pts.split(',')
        self.src_leftx, self.src_rightx, self.laney, self.offsety = int(
            src_leftx), int(src_rightx), int(laney), int(offsety)
        
        self.val_thres_percentile = args.val_thres_percentile
        self.dilate_size = args.dilate_size
        self.sat_cdf_lower_thres = args.sat_cdf_lower_thres
        self.blue_red_diff_thres = args.blue_red_diff_thres
        self.red_val_tolerance = args.red_val_tolerance
        self.window_height = args.window_height

    # def depth_callback(self, data):
    #     try:
    #         # Convert a ROS image message into an OpenCV image
    #         cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
    #     except CvBridgeError as e:
    #         print(e)
    #     cv_image = cv_image.copy()
    #     cv_image = cv_image * 1000
    #     cv_image = cv_image.astype(np.uint16)
    #     self.depth_image = cv_image
    #     cv2.imwrite("depth_image.png", self.depth_image)
    #     obstacles = np.where(self.depth_image < 0.5, 1, 0)
    #     self.obstacle_max = np.max(self.depth_image)
    #     self.obstacle_min = np.min(self.depth_image)
    #     if np.any(obstacles):
    #         self.obstacle_detected = True
    #     else:
    #         self.obstacle_detected = False

    def laser_callback(self, data):
    # data is a LaserScan message

        # Convert the data to a numpy array for easier manipulation
        laser_scan  = np.array(data.ranges)

        # Select only the forward direction
        # Assuming the laser provides a 360-degree field of view
        forward_scan = laser_scan[len(laser_scan)//2 - 10 : len(laser_scan)//2 + 10]

        # Now you can process the laser scan data as needed
        # For example, you might want to check if there are any obstacles close to the robot
        self.obstacle_indices = np.where(forward_scan < 1)
        

        self.obstacle_detected = np.any(forward_scan < 1)

      
    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        start_time = time.time()
        
        raw_img = cv_image.copy()
        ret = self.detection(raw_img)
        print("Detection takes time: {:.3f} seconds".format(time.time() - start_time))

        
        latest_way_pts, latest_reach_boundary = self.get_latest_info()
        self.run(latest_way_pts, latest_reach_boundary)
                

    
    def line_fit(self, binary_warped):
        """""
            lane lines
        """""
        ### sliding window
        h, w = binary_warped.shape
        offset = 5
        margin = 70
        best_base_x = -1
        best_num_pixels = 0

        for cur_x in range(margin, w - margin, offset):
            left_lane = cur_x - margin
            right_lane = cur_x + margin

            total_num = cv2.countNonZero(binary_warped[ -self.window_height:, left_lane:right_lane ])

            if total_num > best_num_pixels:
                best_num_pixels = total_num
                best_base_x = cur_x
        if(best_base_x == -1):
            print("window didn't update")
            return None


        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        # set minimum number of pixels found to window
        minpix = 200
        # convert to gray then convert to white

        color_warped = cv2.cvtColor(binary_warped * 255, cv2.COLOR_GRAY2BGR)
        # make zero pixels to white
        color_warped[color_warped > 0] = 255

        # optimize the base x
        cur_x = best_base_x
        lane_pts = []
        prev_x = []

        i = 0
        reach_boundary = False
        while True:
            ## iterate window get cur top bottom
            
            top = h - (i+1) * self.window_height
            bottom = top + self.window_height

            #adjust base
            # basing on the previous 2 position of x    use slope to make it curvature

            if (i >= 2):
                slope = prev_x[-1] - prev_x[-2]
                cur_x = prev_x[-1] + slope

            window_idx = np.where( (nonzerox < cur_x + margin) & (nonzerox > cur_x - margin) & (nonzeroy > top) & (nonzeroy < bottom))

            ## get the none zero  pixels within the window

            x_window = nonzerox[window_idx]
            y_window = nonzeroy[window_idx]
            i += 1
            #print(x_window)
            #print(y_window)

            reach_boundary = (top < 0) | (cur_x -margin < 0) | (cur_x + margin >= w)

            if len(x_window) < minpix: break

            # get average value in the window    ->    store as way points

            cur_x = int(np.mean(x_window))
            cur_y = int(np.mean(y_window))
            lane_pts.append([cur_x, cur_y])
            prev_x.append(cur_x)
            #print(i)
            ## for visualization
            color_warped = cv2.rectangle(color_warped, (cur_x - margin, top), (cur_x + margin, bottom), (0,0,255))                
        # print(len(lane_pts))
        lanex = [pt[0] for pt in lane_pts]
        laney = [pt[1] for pt in lane_pts]
        # print(np.min(laney))
        
        for x, y in zip(lanex, laney):
            color_warped = cv2.circle(color_warped, (x, y), 1, (0,255,0), -1)
        ret = {}
        ret["vis_warped"] = color_warped
        ret["lanex"] = lanex
        ret["laney"] = laney
        ret["reach_boundary"] = reach_boundary

        return ret
    
    def color_thresh(self, img):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        # 1. Convert the image from RGB to HSL
        # 2. Apply threshold on S channel to get binary image
        # Step 1: Filter out pixels with strong reflection
        img = img.copy()
        blue_channel = img[:, :, 0].astype(np.float32)
        red_channel = img[:, :, 2].astype(np.float32)
        # blud_red_diff_cond = red_channel - blue_channel > self.blue_red_diff_thres

        hls_img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        

        # Step 2: Apply threshold on the S (Saturation) channel to get a binary image
        h, l, s = cv2.split(hls_img)
        binary_output = np.zeros_like(l)
        
        # dynamic search sat_thres_min
        s_warped, M, Minv = self.perspective_transform(s)
        sat_hist, bins = np.histogram(s_warped.flatten(), bins=256, range=[0, 256])

        # Calculate the cumulative distribution function (CDF) of the histogram
        cdf = sat_hist.cumsum()
        cdf_normalized = cdf / cdf.max() # Normalize the CDF to the range [0, 1]
        bin_idxs = \
            np.where((cdf_normalized > self.sat_cdf_lower_thres) & (cdf_normalized < 0.90))[0]
        sat_thres_min = np.argmin( [sat_hist[idx] for idx in bin_idxs] ) + bin_idxs[0]
        sat_cond = ((sat_thres_min <= s) & (s <= 255))
        
        
        # Steps 2: Apply value threshold on image
        # Use red channel of raw_image instead of l channel to do the value filtering
        # Because red channel for yellow lane is much different from background
        red_channel = img[:, :, 2] # red channel
        red_channel_warped, M, Minv = self.perspective_transform(red_channel)
        val_thres_min = np.percentile(red_channel_warped, self.val_thres_percentile)
        # val_mean = np.mean(red_channel_warped)
        val_cond = (val_thres_min <= red_channel) & (red_channel <= 255)

        mean_red_val = np.mean(red_channel_warped)
        mean_red_val_cond = mean_red_val + self.red_val_tolerance <= red_channel    
    
        # Step 3: Apply predefined hue threshold on image
        hue_cond = (self.hue_thres_min <= h) & (h <= self.hue_thres_max)
        
        # combine conditions and get final output
        binary_output[val_cond & sat_cond & hue_cond &  mean_red_val_cond] = 1

        # closing
        kernel = np.ones((self.dilate_size, self.dilate_size), np.uint8)
        binary_output = cv2.morphologyEx(
            binary_output.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

        return binary_output

    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """

        # Define four points as (x, y) coordinates
        src_height, src_width = img.shape[:2]

        
        # dst_width, dst_height = 720, 1250
        dst_width, dst_height = src_width, src_height
        

        pt_A = [115, 294]
        pt_B = [0, src_height]
        pt_C = [src_width, src_height]
        pt_D = [525, 294]

        pt_A1 = [0, 0]
        pt_B1 = [0, dst_height]
        pt_C1 = [dst_width, dst_height]
        pt_D1 = [dst_width, 0]

        input_pts = np.float32([pt_A, pt_B, pt_C, pt_D])
        output_pts = np.float32([pt_A1, pt_B1, pt_C1, pt_D1])
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        M = cv2.getPerspectiveTransform(input_pts,output_pts)
        #3. Generate warped image in bird view using cv2.warpPerspective()
    
        warped_img = cv2.warpPerspective(img,M,(dst_width, dst_height),flags=cv2.INTER_LINEAR)
        Minv = np.linalg.inv(M)
    
        return warped_img, M, Minv


    def get_latest_info(self):
        return self.way_pts, self.reach_boundary
    
    def update_waypoints(self, lanex, laney, width, height, reach_boundary):
         
            # transform from image coord (x, y) to camera coord in meters
            PIX2METER_X = 0.0009525 # meter
            PIX2METER_Y = 0.0018518 # meter
            lanex = [(-x/907.5 + 0.42) * 0.7 for x in lanex]
            laney = [y / 907 + 0.95 for y in laney]
            
            
            way_pts = [(y,x) for x, y in zip(lanex, laney)]
            
            # only update way pts when there are at least 3 points
            ## 3 pts for lane following

            if len(way_pts) >= 3:
                self.way_pts = way_pts
                self.reach_boundary = reach_boundary
                # self.obstacle_detected = False
            else:
                # if(self.obstacle_MUX):
                #     self.obstacle_detected = True
                #     # print("no way_pts")
                # self.way_pts = way_pts
                print ('Number of detected way_pts < 3. Use waypoints of previous frame')
   
    def get_matrix_calibration(self, img_shape,
                               src_leftx=218,
                               src_rightx=467,
                               laney=348,
                               offsety=0):
        """
        Get bird's eye view from input image
        """
        # 1. Visually determine 4 source points and 4 destination points
        # 2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        # 3. Generate warped image in bird view using cv2.warpPerspective()

        # Define four points as (x, y) coordinates
        src_height, src_width = img_shape

        src_pts = np.array([[src_leftx, laney],
                            [0, src_height - offsety],
                            [src_width, src_height - offsety],
                            [src_rightx, laney]], dtype=np.int32)

        dst_width, dst_height = src_width, src_height
        dst_pts = np.array([[0, 0],
                            [0, dst_height],
                            [dst_width, dst_height],
                            [dst_width, 0]], dtype=np.int32)

        def calc_warp_points():
            src = np.float32(src_pts)
            dst = np.float32(dst_pts)
            return src, dst

        src, dst = calc_warp_points()
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)

        return M, Minv

    def convert2CalibrationCoord(self, shape, lanex, laney, Minv):
        num_waypts = len(lanex)
        coords = np.zeros((num_waypts, 2))
        for i, (x, y) in enumerate(zip(lanex, laney)):
            coords[i][0] = x
            coords[i][1] = y

        one_vec = np.ones((num_waypts, 1)) # convert to homogeneous coord
        coords = np.concatenate((coords, one_vec), axis=1)
        
        raw_coords = Minv @ coords.T
        clb_coords_normalize = raw_coords / raw_coords[2]
        clb_coords_normalize = clb_coords_normalize.T
         # Adjust origin to center of the image
        # raw_coords[0] -= 320  # Subtract half of image width
        # raw_coords[1] = 480 - raw_coords[1]  # Subtract half of image height and invert y-axis
        raw_coords[0] = clb_coords_normalize[:, 0]
        raw_coords[1] = clb_coords_normalize[:, 1]
        clb_coords_normalize[:, 0] -= 320
        clb_coords_normalize[:, 1] = 480 - clb_coords_normalize[:, 1]
        

        return raw_coords[0], raw_coords[1], clb_coords_normalize[:, 0], clb_coords_normalize[:, 1]

    def detection(self, img):

        binary_img = self.color_thresh(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)
        ret = self.line_fit(img_birdeye)

        if ret is not None:
            '''''
            1. use calibration coords 
            2. get way pts

            '''''
            raw_x, raw_y, cx, cy = self.convert2CalibrationCoord(img.shape[:2], ret['lanex'], ret['laney'], Minv)
            h, w = img.shape[:2]

            self.update_waypoints(cx, cy, w, h, ret['reach_boundary'])
        
            return ret['reach_boundary'], ret['vis_warped'], cv2.cvtColor(img_birdeye, cv2.COLOR_GRAY2BGR), self.way_pts
        else:
            print("Unable to detect")

            return None

    

    def get_steering_based_point(self, path_points, vision_range=None):
        """
        Calculate the optimal waypoint ahead based on the vehicle's trajectory.
        """
        x_points, y_points = [], []
        for x, y in path_points:
            x_points.append(x)
            y_points.append(y)

        # Set the vision range if not explicitly provided
        if vision_range is None:
            vision_range = self.look_ahead
        # Fit a polynomial to the path points
        path_fit = np.polyfit(x_points, y_points, deg=2)

        # Initialize the target waypoint
        optimal_waypoint = [x_points[0], -1]

        # Start searching from the first point
        search_x = x_points[0]
        # Continue updating until the optimal waypoint is found within vision range
        while True:
            search_y = np.polyval(path_fit, search_x)
            dx = search_x - x_points[0]
            dy = search_y - y_points[0]
            distance_from_start = (dx**2 + dy**2)**0.5

            # Check if the current distance is within the vision range, continue if true
            if distance_from_start <= vision_range:
                optimal_waypoint = [search_x, search_y]
                search_x += 0.01  # Increment the search point
            else:
                break  # Exit the loop when the distance exceeds the vision range

        return optimal_waypoint





    def run(self, way_pts = None, reach_boundary = True):

        ##obstacele avoidance
        if (self.obstacle_detected and self.obstacle_avoidance_MUX and not self.obstacle_planning_withoutpts):
                    self.obstacle_planning_withoutpts = True
                    self.obstacle_start_time = time.time()
        
        ##doing avoidance without way points
        if self.obstacle_planning_withoutpts:
            duration_time = time.time()-self.obstacle_start_time
            tar_v = self.vel_min
            if(duration_time < 1):
                ## need to change the steering angle it is fixed right now can be modified to be dynamic according to lidar
                steering = np.radians(10)

            elif(1 < duration_time < 2):
                steering = np.radians(0)
            
            else:
                steering = np.radians(-10)
                if(self.obstacle_detected == False and duration_time > 3):
                    self.obstacle_planning_withoutpts = False
                    self.obstacle_start_time = 0

            steering = round(np.clip(steering, -np.radians(self.angle_limit), np.radians(self.angle_limit)), 3)
            steering_deg = round(np.degrees(steering))

        ## stop the car
        elif(self.obstacle_detected and not self.obstacle_avoidance_MUX):
            tar_v = 0
            steering = 0 
            steering_deg = round(np.degrees(steering))
                    
        else:
            if way_pts is None:
                print("no waypts in run, keep last steering angle")
                tar_v = self.last_vel * 0.8
                steering = self.last_steering * 0.8
            
            else: 
                self.targ_pts = way_pts
                tar_v = self.vel_max # constant speed
                num_pts = len(self.targ_pts)
                id = [0, num_pts // 2, num_pts - 1]
                curvature = 0
                if len(self.targ_pts) >= 3:
                    dx0 = self.targ_pts[id[1]][0] - self.targ_pts[id[0]][0]
                    dy0 = self.targ_pts[id[1]][1] - self.targ_pts[id[0]][1]
                    dx1 = self.targ_pts[id[2]][0] - self.targ_pts[id[1]][0]
                    dy1 = self.targ_pts[id[2]][1] - self.targ_pts[id[1]][1]

                    ddx = dx1 - dx0
                    ddy = dy1 - dy0
                    if(dx1==0 and dy1 == 0):
                        curvature = np.inf

                    else: curvature = abs((dx1*ddy - dy1*ddx) / (dx1**2 + dy1**2) ** (3/2))
                    
                else:
                    curvature = np.inf

                curvature = np.clip(curvature, self.curv_min, self.curv_max)
                tar_v = self.vel_max - (self.vel_max - self.vel_min) * curvature / (self.curv_max - self.curv_min)  ## whats the meaning of the expression

                if(self.look_ahead > 0):
                    ## look ahead
                    self.goal_x, self.goal_y = self.get_steering_based_point(self.targ_pts)

                else:
                    ## use middle points
                    self.goal_x = way_pts[id[1]][0]
                    self.goal_y = way_pts[id[1]][1]

                 
                  
                distance = np.sqrt(self.goal_x**2 + self.goal_y**2)
                alpha = np.arctan2(self.goal_y, self.goal_x)
                a = np.arctan2(self.steering_k * 2 * self.wheelbase * np.sin(alpha) / distance, 1) * self.steering_i
                steering = round(np.clip(a, -np.radians(self.angle_limit), np.radians(self.angle_limit)), 3)
                steering_deg = round(np.degrees(steering))
                self.last_steering = steering
                self.last_vel = tar_v


        # distance_obstacle =np.sqrt(way_pts[-1][0] ** 2 + way_pts[-1][1] ** 2)
        # if(distance_obstacle < 0.9 and len(way_pts) <= 3):
        #     self.obstacle_detected = True
        # else:
        #     self.obstacle_detected = False
        if not self.debug_mode:
            self.drive_msg.header.stamp = rospy.get_rostime()
            self.drive_msg.drive.steering_angle = steering
            self.drive_msg.drive.speed = tar_v
            self.ctrl_pub.publish(self.drive_msg)

            
        # print for debug
        if(self.obstacle_planning_withoutpts): 
            print(round(np.degrees(steering))
)
        else:
            if(self.obstacle_detected):
                msgs = [
                    "steering(deg): {} degree".format(steering_deg),
                    "target_vel: {:.2f}".format(tar_v),
                    "obs_detected: {}".format(self.obstacle_detected),
                    "avoidance_planning: {}".format(self.obstacle_planning_withoutpts)
                    # "min_laney: {}".format(self.laney)
                    
                ]

            else:
                msgs = [
                    "max lookahead: {:.2f} meters".format(distance),
                    "last waypt: ({:.2f}, {:.2f})".format(self.targ_pts[-1][0], self.targ_pts[-1][1]),
                    "target_pt: ({:.2f}, {:.2f})".format(self.goal_x, self.goal_y),
                    "steering(deg): {} degree".format(steering_deg),
                    "target_vel: {:.2f}".format(tar_v),
                    "curvature: {:.2f}".format(curvature),
                    # "obstacle_max: {:.2f}".format(self.obstacle_max),
                    # "obstacle_min: {:.2f}".format(self.obstacle_min),
                    "obs_detected: {}".format(self.obstacle_detected),
                    "alpha: {}".format(alpha)
                    # "min_laney: {}".format(self.laney)
                ]

            print ('\n----- control msgs -----')
            for i, msg in enumerate(msgs):
                print ('{}. {}'.format(i+1, msg))

            return msgs 
        

    


if __name__ == '__main__':
    args = parser.parse_args()
    print ('======= Initial arguments =======')
    params = []
    for key, val in vars(args).items():
        param = f"--{key} {val}"
        print(f"{key} => {val}")
        params.append(param)
    print ('===============================')

    assert args.curv_min <= args.curv_max
    assert args.vel_min <= args.vel_max
    
    rospy.init_node('rgb_track_node', anonymous=True)
    rate = rospy.Rate(15)  # Hz

    lane_detector = LaneDetector(args)
    
    try:
        print ('\nStart navigation...')
        while not rospy.is_shutdown():
            rate.sleep() 
    except rospy.ROSInterruptException:
        pass