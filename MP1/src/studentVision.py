import time
import math
import numpy as np
import cv2
import rospy

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology

import matplotlib.pyplot as plt

class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        # self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        
        #for 0830
        
        # self.sub_image = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.img_callback, queue_size=1)
        
        
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True


    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)


# para set for GEM
# thresh_min=50, thresh_max=100

    def gradient_thresh(self, img, thresh_min=25, thresh_max=100):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        #1. Convert the image to gray scale
        #2. Gaussian blur the image
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        #4. Use cv2.addWeighted() to combine the results
        #5. Convert each pixel to unint8, then apply threshold to get binary image


        ##------------------------gem---------------------------------
        ## TODO
        # #Convert the image to gray scale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #2. Gaussian blur the image
        blur = cv2.GaussianBlur(gray,(5,5),0)
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        sobelx = cv2.Sobel(blur, cv2.CV_64F, 1,0,ksize =1)
        sobely = cv2.Sobel(blur, cv2.CV_64F, 0,1,ksize =1)
        #added = 0.5*sobelx + 0.5*sobely
        added = cv2.addWeighted(sobelx, 0.75, sobely, 0.25, 0)
        #cv2.imshow("added is", added)
        # pd.DataFrame(added).to_csv('sample.csv') 
        scaled_sobel = np.zeros_like(added)
        scaled_sobel[(added >=thresh_min) & (added<= thresh_max)] = 1
        # thresh = cv2.threshold(added, 10,100, cv2.THRESH_BINARY)
        # # #cv2.imshow("scaled sobel is", scaled_sobel*255)
        # # #cv2.imshow("thresh is", thresh)
        
        # # # plt.imshow(scaled_sobel)
        # # # plt.title("g")
        # # # plt.show()
        
        ##----------------0011sync--------------------------------
        
          ## TODO
          
        # plt.imshow(img)
        # plt.title("raw")
        # plt.show()
        # Convert the image to gray scale
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # #2. Gaussian blur the image
        # blur = cv2.GaussianBlur(gray,(5,5),0)
        # #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        # sobelx = cv2.Sobel(blur, cv2.CV_64F, 1,0,ksize =1)
        # sobely = cv2.Sobel(blur, cv2.CV_64F, 0,1,ksize =1)
        # #added = 0.5*sobelx + 0.5*sobely
        # added = cv2.addWeighted(sobelx, 0.9, sobely, 0.1, 0)
        # #cv2.imshow("added is", added)
        # # pd.DataFrame(added).to_csv('sample.csv') 
        # scaled_sobel = np.zeros_like(added)
        # scaled_sobel[(added >=thresh_min) & (added<= thresh_max)] = 1
        # thresh = cv2.threshold(added, 10,100, cv2.THRESH_BINARY)
        # cv2.imshow("scaled sobel is", scaled_sobel*255)
        # cv2.imshow("thresh is", thresh)
        
        # plt.imshow(scaled_sobel)
        # plt.title("g")
        # plt.show()
        ###----------------------------------------
        
        
        ##----------------00484sync--------------------------------
        
        #   ## TODO
        # #Convert the image to gray scale
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # #2. Gaussian blur the image
        # blur = cv2.GaussianBlur(gray,(5,5),0)
        # #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        # sobelx = cv2.Sobel(blur, cv2.CV_64F, 1,0,ksize =1)
        # sobely = cv2.Sobel(blur, cv2.CV_64F, 0,1,ksize =1)
        # #added = 0.5*sobelx + 0.5*sobely
        # added = cv2.addWeighted(sobelx, 0.9, sobely, 0.1, 0)
        # #cv2.imshow("added is", added)
        # # pd.DataFrame(added).to_csv('sample.csv') 
        # scaled_sobel = np.zeros_like(added)
        # scaled_sobel[(added >=5)] = 1
        #thresh = cv2.threshold(added, 10,100, cv2.THRESH_BINARY)
        #cv2.imshow("scaled sobel is", scaled_sobel*255)
        #cv2.imshow("thresh is", thresh)
        
        # # plt.imshow(scaled_sobel)
        # # plt.title("g")
        # # plt.show()
        # ###----------------------------------------
        
        return scaled_sobel

# para set for GEM
# thresh_min=40, thresh_max=100
    def color_thresh(self, img, thresh=(200, 255)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image    
        #Hint: threshold on H to remove green grass
        ## TODO

        ##gem------------------------
        thresh_min = 40
        thresh_max = 100
        hls_img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        


        h_channel = hls_img[:,:,0]
        s_channel = hls_img[:, :, 2]
        l_channel = hls_img[:, :, 1]

        binary_output = np.ones_like(h_channel)
        
        binary_output[s_channel < 30] = 0
        binary_output[l_channel>70] = 1
        binary_output[(h_channel >= thresh_min) & (h_channel <= thresh_max)] = 0
        # # ###
        # plt.imshow(img)
        # cv2.imwrite("0830_clip.png",img)
        # # print(img.shape)
        # plt.show()


        #0011_sync  /// This is perfect for 0056_sync-------------------- 2.20 7:00pm 
        # thresh_min = thresh[0]
        # thresh_max = thresh[1]
        # hls_img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        


        # h_channel = hls_img[:,:,0]
        # s_channel = hls_img[:, :, 2]
        # l_channel = hls_img[:, :, 1]
        # # print(l_channel)
        # binary_output = np.ones_like(h_channel)
        
       
        # binary_output[l_channel<200] = 0
        # # binary_output[l_channel>= 245] = 0
        # binary_output[(h_channel >= 200) & (h_channel <= 255)] = 0
        # binary_output[s_channel < 20] = 0
        
        ###-----------------------------
        # 0011 
        # thresh_min = thresh[0]
        # thresh_max = thresh[1]
        # hls_img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        


        # h_channel = hls_img[:,:,0]
        # s_channel = hls_img[:, :, 2]
        # l_channel = hls_img[:, :, 1]
        # # print(l_channel)
        # binary_output = np.ones_like(h_channel)
        
       
        # binary_output[l_channel<220] = 0
        # # binary_output[l_channel>= 245] = 0
        # binary_output[(h_channel >= 200) & (h_channel <= 255)] = 0
        # binary_output[s_channel < 60] = 0
        ###------------------------
        
        # ##0484-----------------
        # thresh_min = thresh[0]
        # thresh_max = thresh[1]
        # hls_img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        


        # h_channel = hls_img[:,:,0]
        # s_channel = hls_img[:, :, 2]
        # l_channel = hls_img[:, :, 1]
        # # print(l_channel)
        # binary_output = np.ones_like(h_channel)
        
       
        # binary_output[l_channel>150] = 0
        # binary_output[s_channel>30] = 0
        # binary_output[(h_channel >= 38) & (h_channel <= 360)] = 0
        # threshold_l = 130
        # threshold_h = 150
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # binary_output = np.ones_like(gray)
        # binary_output[gray>threshold_h] = 0
        # binary_output[gray<threshold_l] = 0
        # ##-----------------------------------
        #0830
        # plt.imshow(img)
        # plt.title("g")
        # plt.show()
        
        # thresh_min = thresh[0]
        # thresh_max = thresh[1]
        # hls_img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        
        # # plt.imshow(hls_img)
        # # plt.title("g")
        # # plt.show()
        


        # h_channel = hls_img[:,:,0]
        # s_channel = hls_img[:, :, 2]
        # l_channel = hls_img[:, :, 1]
        # # print(l_channel)
        # binary_output = np.ones_like(h_channel)
        
       
        
        # # # binary_output[l_channel>= 245] = 0
        # binary_output[(h_channel >= 40) & (h_channel <= 100)] = 0
        # binary_output[s_channel < 30] = 0
        # binary_output[l_channel>128] = 1
       
        
        return binary_output


    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        ## TODO

        ####gem use default
        SobelOutput = self.gradient_thresh(img,thresh_min=25, thresh_max=110)
        ColorOutput = self.color_thresh(img)

        binaryImage = np.zeros_like(SobelOutput)
        binaryImage[(ColorOutput==1)|(SobelOutput==1)] = 1
        # Remove noise from binary image
        # print(binaryImage.dtype)
        # binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)
        
        
        ##0083
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=25,connectivity=2)
        # plt.imshow(binaryImage)
        # plt.show()
        
        
 
        return binaryImage.astype('float64')


    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        #3. Generate warped image in bird view using cv2.warpPerspective()

        ## TODO

        
        
        dim= np.shape(img)
        # print(dim)
        height = dim[0]
        width  = dim[1]

        ###------wrong1  half
        # pt_A = [242,276]
        # pt_C = [11,405]
        # pt_B = [397,254]
        # pt_D = [638,318]


        ###gem-----------
        # pt_A = [120,300]
        # pt_B = [500,300]
        # pt_C = [11,450]
        # pt_D = [638,450]
        #------------------
        # pt_A = [150,270]
        # pt_B = [420,270]
        # pt_C = [0,450]
        # pt_D = [630,450]
        
        # potential index---------------
        # pt_A = [200,300]
        # pt_B = [430,300]
        # pt_C = [0,410]
        # pt_D = [620,410]
        
        
        # pt_A = [100,200]
        # pt_B = [400,200]
        # pt_C = [11,450]
        # pt_D = [638,450]
        
        # test para_ xuan  work well on corner
        # pt_A = [130,280]
        # pt_B = [430,280]
        # pt_C = [18,380]
        # pt_D = [638,380]
        
        # test para_ xuan ------correct
        # pt_A = [160,280]
        # pt_B = [430,280]
        # pt_C = [0,390]
        # pt_D = [638,380]
        
        
        pt_A = [190,275]
        pt_B = [380,270]
        pt_C = [0,400]
        pt_D = [631,393]
        #------------------------------------------------------------Rosbag Para
        ##0011 sync
        # pt_A = [520,230]
        # pt_B = [710,230]
        # pt_C = [250,360]
        # pt_D = [770,340]
        #-----------------------------------------------
        #0056sync
        # pt_A = [450,250]
        # pt_B = [720,260]
        # pt_C = [150,380]
        # pt_D = [820,360]
        ##-----------------------------------------------
        #0484
        #
        # pt_A = [530,300]
        # pt_B = [830,320]
        # pt_C = [405,360]
        # pt_D = [900,360]
        #
        # pt_A = [560,300]
        # pt_B = [830,320]
        # pt_C = [420,360]
        # pt_D = [900,360]
        
        ##3
        
        #-------------------------------------------------
        #0830 para
        # pt_A = [470,250]
        # pt_B = [710,260]
        # pt_C = [250,500]
        # pt_D = [900,500]
        
        # pt_A = [520,385]
        # pt_B = [700,385]
        # pt_C = [160,670]
        # pt_D = [1100,670]
        
        input_pts = np.float32([pt_A, pt_B, pt_C, pt_D])
        dim= np.shape(img)
        pt_A1  = [0, 0]
        pt_B1  = [width-1,0]
        pt_C1  = [0,height-1]
        pt_D1  = [width-1, height-1]

     
        output_pts = np.float32([pt_A1, pt_B1, pt_C1, pt_D1])
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        M = cv2.getPerspectiveTransform(input_pts,output_pts)
        #3. Generate warped image in bird view using cv2.warpPerspective()
        # plt.imshow(img)
        # plt.show()binedBinaryImage
        warped_img = cv2.warpPerspective(img,M,(width, height),flags=cv2.INTER_LINEAR)
        Minv = np.linalg.inv(M)
        # plt.imshow(warped_img)
        # plt.show()

        return warped_img, M, Minv


    def detection(self, img):
        ##gem use default parameter
        binary_img = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)

        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            left_fit = ret['left_fit']
            right_fit = ret['right_fit']
            nonzerox = ret['nonzerox']
            nonzeroy = ret['nonzeroy']
            left_lane_inds = ret['left_lane_inds']
            right_lane_inds = ret['right_lane_inds']

        else:
            # Fit lane with previous result
            if not self.detected:
                ret = line_fit(img_birdeye)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                    self.detected = True

            else:
                left_fit = self.left_line.get_fit()
                right_fit = self.right_line.get_fit()
                ret = tune_fit(img_birdeye, left_fit, right_fit)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                else:
                    self.detected = False

            # Annotate original image
            bird_fit_img = None
            combine_fit_img = None
            if ret is not None:
                bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
                combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
            else:
                print("Unable to detect lanes")

            return combine_fit_img, bird_fit_img


if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
