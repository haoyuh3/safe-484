import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode


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

        ##### TODO:  #####
        # Modify the initial particle distribution to be within the top-right quadrant of the world, and compare the performance with the whole map distribution.
        for i in range(num_particles):

            # # (Default) The whole map
            # x = np.random.uniform(0, world.width)
            # y = np.random.uniform(0, world.height)


            ## first quadrant
            x = np.random.uniform(world.width/2, world.width)
            y = np.random.uniform(world.height/2, world.height)


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
        self.particle_weight_total = 0
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
        Returns:
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
        sum = 0
        for particle in self.particles:
            # print(particle.read_sensor(), readings_robot)
            sensor_particle = particle.read_sensor()
            particle.weight = self.weight_gaussian_kernel(readings_robot, sensor_particle)
            sum += particle.weight
            # weight.append(particle.weight)
        # print(sum)
        for i in range(len(self.particles)):
            self.particles[i].weight /= sum
        
        
        
         
        ###############
        # pass

    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        ## TODO #####
        particles_new = list()
        sum = np.zeros(len(self.particles))
        sum[0] = self.particles[0].weight
        for i in range(1, len(self.particles)):
            sum[i] = sum[i-1] + self.particles[i].weight
        
        while len(particles_new) < len(self.particles):
            random_num = np.random.rand()
            # print(random_num)
            idx = 0
            while (idx < 1000 and sum[idx] < random_num ):
                idx += 1
            idx -= 1
            particles_new.append(self.particles[idx])
        
        
       

        ###############

        self.particles = particles_new

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
        """
        ## TODO #####
        dt = 0.01
        if(len(self.control)>0):
            v, delta = self.control[-1]
            
            for particle in self.particles:
                initial = [particle.x, particle.y, particle.heading]
                r = ode(vehicle_dynamics, jac = None)
                r.set_initial_value(initial, 0)
                r.set_f_params(v, delta)
                ret = r.integrate(r.t+dt)
                particle.x = ret[0]
                particle.y = ret[1]
                particle.heading = ret[2]
        

        ###############
        # pass


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        count = 0 
        while True:
            ## TODO: (i) Implement Section 3.2.2. (ii) Display robot and particles on map. (iii) Compute and save position/heading error to plot. #####
            self.particleMotionModel()
            read_vehicle = self.bob.read_sensor()
            self.updateWeight(read_vehicle)
            self.resampleParticle()
            # print(len(self.particles))
            self.world.clear_objects()
            self.world.show_maze()
            self.world.show_particles(self.particles)
            self.world.show_robot(self.bob)
            self.world.show_estimated_location(self.particles)
            
            ###############
            
        