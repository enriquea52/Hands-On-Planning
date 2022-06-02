#!/usr/bin/python3

from turtle import distance
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import numpy as np
from sklearn.cluster import DBSCAN

from utils_lib.exploration import frontier_detection
from utils_lib.bresenham import bresenham_line


class exploration_node:

    # OnlinePlanner Constructor
    def __init__(self, gridmap_topic, odom_topic, scan_topic, laser_angle_offset):
      
        self.current_pose = None
                                             
        self.last_map_time = rospy.Time.now()

        # ROS Subscribers

        self.gridmap_sub = rospy.Subscriber(gridmap_topic, OccupancyGrid, self.get_gridmap)  # Subscriber to gridmap_topic from Octomap Server  
        
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom)                # Subscriber to odom_topic  
        
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.get_scan)               # Subscibe to scan_topic of LaserScan type and call get_scan when new data is available
        
        self.explore_sub = rospy.Subscriber("/continue_exploring", Bool, self.find_nbv)      # Subscibe to scan_topic of LaserScan type and call get_scan when new data is available

        # ROS publishers
        # visualization publishers

        self.frontier_pub = rospy.Publisher('~frontier_marker', Marker, queue_size=1)     # frontier visualization

        self.cells_pub = rospy.Publisher('~individual_cell_marker', Marker, queue_size=1) # detected cells visualization

        self.centroid_pub = rospy.Publisher('~centroid_marker', Marker, queue_size=1)     # centroid visualization

        self.nbv_pub = rospy.Publisher('~nvb_pose', Marker, queue_size=1)                 # nbv_pose visualization

        self.NBV_pub = rospy.Publisher('/exploration', PoseStamped, queue_size=1)         # Publisher for the NBV where the robot has to go

        self.laser_angle_offset = laser_angle_offset


        # ROS timers
        rospy.Timer(rospy.Duration(2), self.NBV)

        #######################################################################
        self.frontier_detector = frontier_detection(self.laser_angle_offset)  #  
        self.grid_resolution = None                                           #
        self.grid_origin = None                                               #
        self.grid_map = None                                                  #
        #######################################################################

        # Frontier Storage
        self.frontiers = None

        # Safety flags for avoiding data collision problems

        self.NBV_flag = True

        self.next_best_view = None


        self.last_scan_time = rospy.Time.now()

    def find_nbv(self, msg):
        # Retrieve command from the planning node, to send another NBV
        self.NBV_flag = True


    def get_odom(self, odom):
        # Subroutine to get odometry readings of the robot
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                            odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z,
                                                            odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

    def get_gridmap(self, gridmap):
      
        # Get the gridmap from the Octomap server
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:      

            self.last_map_time = gridmap.header.stamp

            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T.astype(int)

            self.grid_resolution = gridmap.info.resolution

            self.grid_origin = np.asarray([gridmap.info.origin.position.x, gridmap.info.origin.position.y])

            self.grid_map = env

    def get_scan(self, scan):
        # Get a reading of the RPLiDAR to update the frontiers

        if (scan.header.stamp - self.last_scan_time).to_sec() > 1:  
            
            self.last_scan_time = scan.header.stamp
            
            if self.grid_map is not None:

                sensor_readings = []

                # For each laser beam in the scan message, call gridmap.add_ray function
                x = self.current_pose[0]
                y = self.current_pose[1]
                yaw = self.current_pose[2]

                for i, r in enumerate(scan.ranges):
                    
                    # If the laser range is in the range of the sensor apply the update
                    yaw_inc = i*scan.angle_increment

                    if r >= scan.range_min and r <= scan.range_max:  

                        sensor_readings.append([r, yaw_inc])

                    else:

                        sensor_readings.append([scan.range_min, yaw_inc])

                lr = np.asarray(sensor_readings)

                # Call Fast Frontier Detector Algorithm to detect frontiers and store them  
                self.frontiers, _ = self.frontier_detector.detect_frontiers(lr, self.current_pose, self.grid_origin, self.grid_resolution, self.grid_map)

                # Visualied the frontiers 
                self.publish_frontiers(self.frontiers)

                #self.publish_detected_cells(vis_cells) # Visualization of the laser scans from the robot for debugging purposes

                if self.frontiers is not None and self.frontiers.shape[0] < 150:
                    print("EXPLORATION HAS FINISHED")
                    exit()


    def publish_frontiers(self, frontiers):
        # Publish frontiers as red cubes
        if len(frontiers) > 1:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = 6
            m.ns = 'frontiers'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.frontier_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = 0.05
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            color_red = ColorRGBA()
            color_red.r = 1
            color_red.g = 0
            color_red.b = 0
            color_red.a = 1
            color_blue = ColorRGBA()
            color_blue.r = 0
            color_blue.g = 0
            color_blue.b = 1
            color_blue.a = 1

            
            for n in frontiers:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.frontier_pub.publish(m)


    def publish_detected_cells(self, cells):
        # Publish detected cells as blue cubes fr debugging purposes

        if len(cells) > 1:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = 6
            m.ns = 'cells'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.cells_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = 0.05
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            color_red = ColorRGBA()
            color_red.r = 1
            color_red.g = 0
            color_red.b = 0
            color_red.a = 1
            color_blue = ColorRGBA()
            color_blue.r = 0
            color_blue.g = 0
            color_blue.b = 1
            color_blue.a = 1

            
            for n in cells:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_blue)
            
            self.cells_pub.publish(m)

    def NBV(self, event):
        # main routine to deterimne the Next Best View

        if self.NBV_flag:

            if self.frontiers is None:
                return

            self.NBV_flag = False

            centroids = self.frontier_clustering()

            self.publish_cluster_centroids(centroids)

            self.next_best_view = self.determine_next_best_view(centroids)

            # NBV marker visualization
            self.visualize_nbv(self.next_best_view)

            # Send NBV to path planner
            NBV_PoseStamped = PoseStamped()
            NBV_PoseStamped.pose.position.x = self.next_best_view[0]
            NBV_PoseStamped.pose.position.y = self.next_best_view[1]
            NBV_PoseStamped.pose.position.z = 0.0
            NBV_PoseStamped.pose.orientation.w = 1
            self.NBV_pub.publish(NBV_PoseStamped)
            rospy.sleep(2.)

    def frontier_clustering(self):
        # Compute DBSCAN this is the core of the algorithm
        
        X = self.frontiers                                # Store the dat, which  in this case is the xy coordinates of the frontiers
        
        db = DBSCAN(eps=0.2, min_samples=10).fit(X)       # Deploy the DBSCAN algorithm
        labels = db.labels_                               # Get the clabels fron the clustering algorithm

        unique_labels = np.unique(labels)                 # Get unque label, for iterating over each cluster
        unique_labels = np.delete(unique_labels, np.argwhere(unique_labels < 0))    # delete noise labels equal to -1

        # Container to store the centroid of each cluster
        centroid = np.empty((1, 2))

        # Get the centroid of each cluster and store it 
        for k in unique_labels:
            cluster_indexes = np.argwhere(labels == k)
            xy = X[cluster_indexes]
            if xy.shape[0] > 0:
                centroid = np.vstack((centroid, np.sum(xy, axis=0)/xy.shape[0]))

        return centroid

    def publish_cluster_centroids(self, centroids):
        # Publish clustered frontier centroids as yellow spheres

        if len(centroids) > 1:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = 7          #sphere list
            m.ns = 'centroids'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.frontier_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1

            # Defining the yellow color
            color_yellow = ColorRGBA() 
            color_yellow.r = 230/255
            color_yellow.g = 255/255
            color_yellow.b = 110/255
            color_yellow.a = 1

            

            
            for n in centroids:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_yellow)
            
            self.centroid_pub.publish(m)
        
    def distance_to_sample(self, poses):
        # compute the distance betweent he current robot's position and every sample

        distances = np.linalg.norm(poses - self.current_pose[0:2], axis = 1).reshape(-1, 1)
        
        return distances

    def determine_next_best_view(self, centroids):
        # determine the NBV based onthe information gain of multiple samples around the centroids of the forntiers

        current_entropy = self.view_entropy(self.__position_to_map__(self.current_pose[0:2].reshape(1,2))) # compute the current entroy given the currents robot's pose
        expected_entropy = np.empty((1,1))                        # storage for the expected entropy for the artificial views
        views_poses = np.empty((1,2))                             # storage for the expected entropy for the artificial views

        for centroid_pose in centroids:

            sampled_poses = self.samples_around_frontiers(centroid_pose)

            for pose in sampled_poses:

                cell = self.__position_to_map__(pose)

                if cell[0] >= 0 and cell[1] >= 0 and cell[0] < self.grid_map.shape[0] and cell[1] < self.grid_map.shape[1] and self.grid_map[cell[0], cell[1]] == 0 and np.linalg.norm(pose - self.current_pose[0:2]) > 0.8:  
                    #expected_entropy = np.vstack((expected_entropy,self.view_entropy(cell)))
                    expected_entropy = np.vstack((expected_entropy,self.view_entropy(self.sensor_simulator(pose))))
                    views_poses = np.vstack((views_poses, pose))
        
        # deleting empty containers
        expected_entropy = np.delete(expected_entropy, 0, axis = 0); views_poses = np.delete(views_poses, 0, axis=0)
        
        information_gain = current_entropy - expected_entropy

        k_gain = 100

        distance_to_every_view = self.distance_to_sample(views_poses)*k_gain

        total_gain = information_gain/distance_to_every_view


        next_best_view = np.round(views_poses[np.argmin(total_gain)], 3)

        return next_best_view

    def view_entropy(self, cell, window_size = 5): 
        # Compute the entropy of a sampled view

        # sampled information pre-processing
        information = self.grid_map[cell[:,0], cell[:,1]]
        information = information / 100
        information = np.where(information < 0, 0.5, information)
        information = np.where(information == 0, 1, information)

        # compute entropy gain for the given sampled window
        entropy = -np.sum(information * np.log(information))

        return entropy

    def __position_to_map__(self, p): 
        # convert world position to map coordinates

        uv = (p - self.grid_origin) / self.grid_resolution

        return uv.astype(int)

    def samples_around_frontiers(self, pose, margin = 0.5, number_of_samples = 50):
        # get 'number_of_samples' samples of positions around a given position in cell format

        samples_around_pose = np.random.uniform(low=-margin, high=margin, size=(number_of_samples, 2)) + pose

        return samples_around_pose

    def sensor_simulator(self, start, range_z = 2, samples = 60, angle_range = 2*np.pi):
        # Simulate a sensor reading (RPLiDAR scan) for every sample

        angles = np.linspace(0, angle_range, samples )

        ends = np.array([start[0]+(range_z*np.cos(angles)), start[1]+(range_z*np.sin(angles))]).T

        cell_start = self.__position_to_map__(start) 
        cell_ends = self.__position_to_map__(ends)    

        simulated_scan = np.zeros((1, 2))


        for cell_end in cell_ends:

            line =  np.asarray(bresenham_line(cell_start,cell_end))

            for cell in line:

                if cell[0] < 0 or cell[1] < 0 or cell[0] >= self.grid_map.shape[0] or cell[1] >= self.grid_map.shape[1] or self.grid_map[cell[0], cell[1]] > 50:
                    break
            
                simulated_scan = np.vstack((simulated_scan, cell))

        simulated_scan = np.delete(simulated_scan, 0, axis = 0)
                
        return simulated_scan.astype(int)

    def visualize_nbv(self, nbv):
        # Publish clustered frontier centroids as yellow spheres

        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = rospy.Time.now()
        m.id = 0
        m.type = Marker.ARROW         
        m.ns = 'nbv'
        m.action = Marker.DELETE
        m.lifetime = rospy.Duration(0)
        self.nbv_pub.publish(m)

        m.action = Marker.ADD
        m.scale.x = 0.02
        m.scale.y = 0.04
        m.scale.z = 0.03
        
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1

        m.color.r = 27/255
        m.color.g = 163/255
        m.color.b = 156/255
        m.color.a = 1

        start = Point()
        start.x = nbv[0]
        start.y = nbv[1]
        start.z = 0

        end = Point()
        end.x = nbv[0]
        end.y = nbv[1]
        end.z = 0.5

        m.points.append(end)
        m.points.append(start)

        self.nbv_pub.publish(m)


if __name__ == '__main__':

    print("EXPLORATION NODE IGNITED")

    rospy.init_node('exploration_node')   

    # Required topics
    gridmap_topic = rospy.get_param("/octomap_topic")
    odometry_topic = rospy.get_param("/tbot_odometry_topic")
    scan_topic = rospy.get_param("/tbot_rplidar_scan_topic")
    laser_angle_offset = rospy.get_param("/tbot_rplidar_scan_offset")


    # Initialize exploration node
    node = exploration_node(gridmap_topic, odometry_topic, scan_topic, laser_angle_offset)   

    # Run forever
    rospy.spin()