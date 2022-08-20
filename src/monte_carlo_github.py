#!/usr/bin/env python
import rospy
import tf
import tf.transformations as tr
import rostopic
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt, cos, sin, pi, atan2
from threading import Thread, Lock
from math import pi, log, exp
import random
import numpy as np


## the node is implemented to simulate monte carlo localization

## accommodating the pose of each particle in a class and call them when it needs
class Particle(object):
    def __init__(self, id, x,y, theta):
        self.x = x
        self.y = y
        self.id = id    # the id of each particle
        self.theta = theta

# the following class includes the main computation of monte carlo localization, such as 
#model odometry and observation
class ParticleFilter(object):
    def __init__(self, num_particles, occ_grid_map,L, xmin, xmax, ymin, ymax,
                 laser_min_range, laser_max_range, laser_min_angle, laser_max_angle,
                 translation_noise_std_dev,
                 orientation_noise_std_dev,
                 beam_range_measurement_noise_std_dev):
        

        self.num_particles = num_particles
        self.ogm = occ_grid_map     # the map we load in next class using GetMap
        self.grid_map = np.array(self.ogm.data, dtype='int8')    
        self.grid_map = self.grid_map.reshape((self.ogm.info.height, self.ogm.info.width))  # reshaping the map array(convert 1D array to 2D array)
        self.grid_bin = (self.grid_map == 0).astype('uint8')
        self.map_h=self.ogm.info.height
        self.map_w=self.ogm.info.width
        self.laser_scan_n=360                  # number of beams, it completelly depends on laser scan 
                                                 #(for our use the laser range finder has 360 beams)
                                               

        

        ## L is loaded in next class and includs data of precaching, those data has 120*(120*360) dimention (2D) 
        ## and we reshape it to (120*120*360) (#D dimention). for each grid cell (each pose) we calculates the ranges that robot 
        ## would observe if it has located at that position.
        self.precaching_map_data=np.reshape(L, (self.map_h,self.map_w,self.laser_scan_n))
        
        self.max_range=3.5



        # Workspace boundaries
        self.xmax = xmax
        self.xmin = xmin
        self.ymin = ymin
        self.ymax = ymax

        self.laser_max_angle = laser_max_angle
        self.laser_min_angle = laser_min_angle
        self.laser_max_range = laser_max_range
        self.laser_min_range = laser_min_range

        ## noise in translation, oreintation and laser data
        self.translation_noise_std_dev = translation_noise_std_dev
        self.orientation_noise_std_dev = orientation_noise_std_dev
        self.beam_range_measurement_noise_std_dev = beam_range_measurement_noise_std_dev
      
        self.last_robot_odom = None
        self.curr_robot_odom = None
        self.dx = 0
        self.dy = 0
        self.dyaw = 0
        self.theta=0
        self.particles = []
        self.weights = []

    ## using the two following function we randomise the particles over the map
    def random_state(self):
        while True:

            x_rand = np.random.uniform(self.xmin, self.xmax)
            y_rand = np.random.uniform(self.ymin, self.ymax)
            row, col = self.metric_to_grid_coords(x_rand, y_rand)  # the function to convert the pose to grid
            if self.grid_bin[row, col]:
                theta = np.random.uniform(0, 2*pi)
                return x_rand, y_rand, theta

    def initialization(self):

        for i in range(self.num_particles):
            xrand, yrand, theta = self.random_state()

            self.particles.append(Particle(i, xrand,yrand, 0))


    # observation function relies on error between actual laser scan data and the simualted laser scan come from precaching
    #scheme, 
    def observation(self, laser_scan):

        errors = []
      
        for particle in self.particles:
            self.motion_prediction(particle)
            error = self.error_calculation(laser_scan, particle)
            errors.append(error)
          
        
        self.weights = [exp(-error) for error in errors]
        new_particles = []
        self.SUS_resampling(new_particles)
        self.particles = new_particles

    #sus (stochastic universal sampling) proioritize the partciles for next update based on thier wights 
    def SUS_resampling(self, new_particles):
        rand_=np.random.uniform(0,1/self.num_particles)

        if self.weights == []:
            self.weights = [1] * self.num_particles
        
        normalized_weights=[x/sum(self.weights) for x in self.weights]

        c=normalized_weights[0]
        J=0
           
        for m in range (self.num_particles):
                u=rand_+(m)*(1/self.num_particles)
                while u>c:
                    J=J+1
                    c=c+normalized_weights[J]
                    
                particle_ind=self.particles[J]
                new_particles.append(Particle(particle_ind.id, particle_ind.x, particle_ind.y, particle_ind.theta))

    def error_calculation(self, laser_scan_msg, particle):

        if particle.x < self.xmin or particle.x > self.xmax:    #not allowed location
            return 1000

        if particle.y < self.ymin or particle.y > self.ymax:    #not allowed location
            return 1000
 
        row, col = self.metric_to_grid_coords(particle.x, particle.y)   #not allowed location
        if row >= 201 or col >=201:
            return 1000

        if not self.grid_bin[row, col]:
            return 1000

        ranges=laser_scan_msg.ranges
        R_array=np.array(ranges)
        R_array[R_array==np.inf]=self.max_range/2       # manipulating those elements of array with the value of inf
                                                        # to have similar value with precaching data when inf occures
        
        ##Note: manipulating the precaching data to match with actual ranges (to know more detail read github description) 

        if self.theta<0:
          angle=6.28+self.theta
        else:
          angle=self.theta

        D=self.precaching_map_data[col,row,:]
        
        data=np.zeros(self.laser_scan_n)
        
        index=int((angle*self.laser_scan_n)/6.28)  
        
        data[0:self.laser_scan_n-index]=D[index:self.laser_scan_n]
        data[self.laser_scan_n-index:self.laser_scan_n]=D[0:index]


        diff = data-R_array
   
        norm_error = np.linalg.norm(diff)
       
        return norm_error**2

    # calculating the velocites of the robot whenever the odometry subscriber call the odometry callback function

    def velocities(self, robot_odom):

               
    

        self.last_robot_odom = self.curr_robot_odom
        self.curr_robot_odom = robot_odom

        if self.last_robot_odom:

            curr_pose = np.array([self.curr_robot_odom.pose.pose.position.x,
                                           self.curr_robot_odom.pose.pose.position.y,
                                           self.curr_robot_odom.pose.pose.position.z])
            last_pose = np.array([self.last_robot_odom.pose.pose.position.x,
                                           self.last_robot_odom.pose.pose.position.y,
                                           self.last_robot_odom.pose.pose.position.z])

            curr_quat= np.array([self.curr_robot_odom.pose.pose.orientation.x,
                                           self.curr_robot_odom.pose.pose.orientation.y,
                                           self.curr_robot_odom.pose.pose.orientation.z,
                                           self.curr_robot_odom.pose.pose.orientation.w])
            last_quat = np.array([self.last_robot_odom.pose.pose.orientation.x,
                                           self.last_robot_odom.pose.pose.orientation.y,
                                           self.last_robot_odom.pose.pose.orientation.z,
                                           self.last_robot_odom.pose.pose.orientation.w])



            R = tr.quaternion_matrix(last_quat)[0:3,0:3]

            p_lastbaselink_currbaselink = R.transpose().dot(curr_pose - last_pose)
            q_lastbaselink_currbaselink = tr.quaternion_multiply(tr.quaternion_inverse(last_quat), curr_quat)

            rol,pitch, yaw_diff = tr.euler_from_quaternion(q_lastbaselink_currbaselink)

            self.dyaw += yaw_diff
            self.dx += p_lastbaselink_currbaselink[0]
            self.dy += p_lastbaselink_currbaselink[1]
            roll,pitch,theta=tr.euler_from_quaternion(curr_quat)
            self.theta=theta


    def motion_prediction(self, particle):


        noise_x = random.gauss(0, self.translation_noise_std_dev)
        noise_y = random.gauss(0, self.translation_noise_std_dev)
        noise_theta = random.gauss(0, self.orientation_noise_std_dev)

        
        vel=np.sqrt(self.dx**2+self.dy**2)
        if abs(vel) < 1e-10 and abs(vel) < 1e-5:
            return

        particle.x += vel * cos(particle.theta)+noise_x
        particle.y += vel* sin(particle.theta)+noise_y
        particle.theta += self.dyaw + noise_theta


    def metric_to_grid_coords(self, x, y):
        gx = (x - self.ogm.info.origin.position.x) / self.ogm.info.resolution
        gy = (y - self.ogm.info.origin.position.y) / self.ogm.info.resolution
        row = min(max(int(gy), 0), self.ogm.info.height)
        col = min(max(int(gx), 0), self.ogm.info.width)
        return (row, col)

class MonteCarloLocalization(object):

    def __init__(self, num_particles, xmin, xmax, ymin, ymax):
        rospy.init_node('monte_carlo_localization', anonymous=True)

        translation_noise_std_dev   = 0.1
        orientation_noise_std_dev   = 0.01
        beam_range_measurement_noise_std_dev = 0.05
        rospy.wait_for_service("static_map")
        self.grid_map =rospy.ServiceProxy("static_map", GetMap)().map
        self.ogm = self.grid_map

        TH=rospy.get_param("~precaching")
        M_file = open(TH, 'rb')
        L=np.loadtxt(M_file)
        M_file.close()
        self.q_baselink_baselaser = np.array([1.0, 0, 0, 0])
        self.R = tr.quaternion_matrix(self.q_baselink_baselaser)[0:3,0:3]
        self.p_baselink_baselaser = np.array([0.337, 0.0, 0.308])

        self.T= ParticleFilter(num_particles, self.ogm,L,xmin, xmax, ymin, ymax, 0, 0, 0, 0,
                                 translation_noise_std_dev,
                                 orientation_noise_std_dev,
                                 beam_range_measurement_noise_std_dev)

        self.T.initialization()
       
        self.last_scan = None
        self.mutex = Lock()
        
        self.particles_pub = rospy.Publisher('particles', MarkerArray, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odometry_callback, queue_size=1)
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_scan_callback, queue_size=1)
        rate = rospy.Rate(10)

    def odometry_callback(self, msg):
        self.mutex.acquire()
        self.T.velocities(msg)
        self.mutex.release()
              


    def laser_scan_callback(self, msg):

        self.mutex.acquire()
        self.T.observation(msg)
        self.T.dx = 0
        self.T.dy = 0
        self.T.dyaw = 0

        self.mutex.release()
        self.last_scan = msg
        


    def partcile_publisher(self):
       
            self.marker_arr=MarkerArray()
            self.marker_arr.markers=[]
          
            i=0
            for particle in self.T.particles:
                self.marker = Marker()

                self.marker.header.frame_id = "odom"
                ts = rospy.Time.now()
                self.marker.header.stamp = ts
                self.marker.type = self.marker.SPHERE
                self.marker.action = self.marker.ADD
                self.marker.ns="est_pose_"+str(i)
                self.marker.id=i
                self.marker.lifetime = rospy.Duration(1)

                # marker scale
                self.marker.scale.x = 0.04
                self.marker.scale.y = 0.04
                self.marker.scale.z = 0.04

                # marker color
                self.marker.color.a = 1.0
                self.marker.color.r = 1.0
                self.marker.color.g = 0.0
                self.marker.color.b = 0.0

                # marker orientaiton
                self.marker.pose.orientation.x = 0.0
                self.marker.pose.orientation.y = 0.0
                self.marker.pose.orientation.z = 0.0
                self.marker.pose.orientation.w = 1.0
            
                self.marker.pose.position.x=particle.x
                self.marker.pose.position.y=particle.y
                self.marker.pose.position.z=0
                self.marker_arr.markers.append(self.marker)
                i=i+1
              
            self.particles_pub.publish(self.marker_arr)
            


if __name__ == '__main__':
    num_particles = 100
    xmin = -3
    xmax =3
    ymin = -3
    ymax = 3

    mcl = MonteCarloLocalization(num_particles, xmin, xmax, ymin, ymax)
    while not rospy.is_shutdown():
      mcl.partcile_publisher()


