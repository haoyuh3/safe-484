

import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode
import time
import random

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx,dy,dtheta]

class particleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        particles = list()
        self.weights = []

        ##### TODO:  #####
        # Modify the initial particle distribution to be within the top-right quadrant of the world, and compare the performance with the whole map distribution.
        for i in range(num_particles):

            # (Default) The whole map

            x = np.random.uniform(0, world.width)
            y = np.random.uniform(0, world.height)


            # ## first quadrant
            # x = np.random.uniform(world.width/2, world.width)
            # y = np.random.uniform(world.height/2, world.height)
            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))

        ###############

        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
        self.control_len = 0
        return

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:stored
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def weight_gaussian_kernel(self,x1, x2, std = 5000):

        if x1 is None: # If the robot recieved no sensor measurement, the weights are in uniform distribution.
            
            return 1./len(self.particles)
        else:
      
            tmp1 = np.array(x1)
            tmp2 = np.array(x2)
            return np.sum(np.exp(-((tmp2-tmp1) ** 2) / (2 * std)))



    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot 
        Input:
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        """
      
        
        ## TODO #####
       

        particle_weights = np.array([])

        for i in range(len(self.particles)):
            sensor_particle = self.particles[i].read_sensor()
            particle_weights = np.append(particle_weights, self.weight_gaussian_kernel(readings_robot, sensor_particle))

        particle_weights = particle_weights / np.sum(particle_weights)

        return particle_weights
        

    def resampleParticle(self, weight):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        particles_new = list()

        ## TODO #####
        #method 1 --- document
        distribution = list()
        idx = 0
       
        distribution = np.cumsum(weight)
        # total_weight = distribution[-1]
        distribution /= np.sum(distribution)
        for particle in self.particles:
            a = np.random.uniform(0, distribution[-1])
            
            idx = np.searchsorted(distribution,a)
            new_part = Particle(self.particles[idx].x, self.particles[idx].y, self.world, self.particles[idx].heading, weight = weight[idx], sensor_limit = self.sensor_limit, noisy=True)
            particles_new.append(new_part)


        


        self.particles = particles_new



        


    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
        """
        ## TODO #####

        
        dt = 0.01
        
        for j in range(self.control_len, len(self.control)):
   
            for i in range(len(self.particles)):
                arr = vehicle_dynamics(0, [self.particles[i].x, self.particles[i].y, self.particles[i].heading], self.control[j][0],self.control[j][1])
                self.particles[i].x += arr[0]*dt
                self.particles[i].y += arr[1]*dt
                self.particles[i].heading += arr[2]*dt
        self.control_len = len(self.control)
        

        '''
        Motion model with ode()
        '''

        # if len(self.control)>0:
        #     dt = 0.01
        #     #print("self control", self.control[-1][1])
        #     delta = self.control[-1][1]
        #     vr = self.control[-1][0]
        #     # print(vr, delta)
            
        #     for i in range(len(self.particles)):
        #         var = [self.particles[i].x, self.particles[i].y, self.particles[i].heading]
        #         r = ode(vehicle_dynamics)
        #         r.set_initial_value(var,0)
        #         r.set_f_params(vr,delta)
        #         ret = r.integrate(r.t + dt)
        #         self.particles[i].x = ret[0]
        #         self.particles[i].y = ret[1]
        #         self.particles[i].heading = ret[2]
    def euclidean_distance(x1, x2):
        return np.sqrt(np.sum((x1-x2)**2))

    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        count = 0 

        err_dist = np.array([])
        err_heading = np.array([])
        while True:
            ## TODO: (i) Implement Section 3.2.2. (ii) Display robot and particles on map. (iii) Compute and save position/heading error to plot. #####
            self.world.clear_objects()
            

            self.particleMotionModel()
         
            self.resampleParticle(self.updateWeight(self.bob.read_sensor()))
            count += 1
                    
            self.world.show_particles(self.particles, show_frequency=15)
            self.world.show_robot(self.bob)
            e_x, e_y, e_h = self.world.show_estimated_location(self.particles)
            
            b_x = self.bob.x
            b_y = self.bob.y
            b_h = self.bob.heading * 180/np.pi
            
            err_dis =  np.sqrt((b_x - e_x)**2 + (b_y - e_y)**2)
            err_head = np.abs(e_h - b_h)
            if err_head > 180:
                err_head = 360 -err_head
                
            err_dist = np.append(err_dist, err_dis)
            err_heading = np.append(err_heading, err_head)
            # bob_x = np.append(bob_x, self.bob.x)
            # bob_y = np.append(bob_y, self.bob.y)
            # bob_heading = np.append(bob_heading, self.bob.heading)
                
            # estimate_x = np.append(estimate_x, self.world.show_estimated_location(self.particles)[0])
            # estimate_y = np.append(estimate_y, self.world.show_estimated_location(self.particles)[1])
            # estimate_heading = np.append(estimate_heading, self.world.show_estimated_location(self.particles)[2])
   
            
            
            if (count % 5 == 0):
                np.save('error_dist', err_dist)
                np.save('error_orientation', err_heading)
                


            ###############