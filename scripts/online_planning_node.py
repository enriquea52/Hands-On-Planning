#!/usr/bin/python3

import rospy
import numpy as np
import tf
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


import rrt_star_planner as rrt_s_d

from utils_lib.safe_planning import PathValidityChecker

class OnlinePlanner:

    # OnlinePlanner Constructor
    def __init__(self, gridmap_topic, odom_topic, dominion):

        # ATTRIBUTES
        # List of points which define the plan. None if there is no plan
        self.path = []
        # Current robot SE2 pose [x, y, yaw], None if unknown            
        self.current_pose = None
        # Goal where the robot has to move, None if it is not set                                                                   
        self.goal = None
        # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()
        # Dominion [min_x_y, max_x_y] in which the path planner will sample configurations                           
        self.dominion = dominion

        self.PVC = PathValidityChecker()

        self.planner = rrt_s_d.rrt_star_dubins(dominion*2, dominion, dominion, 0.05, 2500)

        # Publisher for visualizing the path to with rviz
        self.path_pub = rospy.Publisher('/path', Path, queue_size=1)
        # Publisher for telling the exploration algorithm to give us another frontier
        self.explore_pub = rospy.Publisher('/continue_exploring', Bool, queue_size=1)

        # SUBSCRIBERS
        self.gridmap_sub = rospy.Subscriber(gridmap_topic, OccupancyGrid, self.get_gridmap)  # : subscriber to gridmap_topic from Octomap Server  
        
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom) # : subscriber to odom_topic  
        
        #self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal) # : subscriber to /move_base_simple/goal published by rviz    
    
        self.move_goal_sub = rospy.Subscriber('/exploration', PoseStamped, self.get_goal, queue_size=1, buff_size=1) # : subscriber to /move_base_simple/goal published by rviz    

        # ROS timers
        #rospy.Timer(rospy.Duration(60), self.new_search)


    def get_odom(self, odom):
        # Odometry callback: Gets current robot pose and stores it into self.current_pose

        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    
    # Map callback:  Gets the latest occupancy map published by Octomap server and check 
    # validity of the path
    def get_gridmap(self, gridmap):
      
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:      

            self.last_map_time = gridmap.header.stamp

            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T.astype(int)

            self.grid_resolution = gridmap.info.resolution

            self.grid_origin = np.asarray([gridmap.info.origin.position.x, gridmap.info.origin.position.y])

            self.grid_map = env

        if self.path is not None and len(self.path) > 0:
            if np.linalg.norm(self.current_pose[0:2] - self.path[-1]) < 0.2:
                self.explore_pub.publish(Bool())

            self.PVC.set(self.grid_map, self.grid_resolution, self.grid_origin)

            if not self.PVC.check_path(self.path):
                #self.robot_enabled(False)
                print("REPLANNING")
                self.path = None
                self.plan(replanning=True)

    def get_goal(self, goal):

        self.goal = np.array([goal.pose.position.x, goal.pose.position.y])

        self.plan()

    def plan(self, replanning = False):
        # Function that produces a path using the RRT* path planning algorithm with Dubin's Path
        
        self.robot_enabled(False)
        print("PLANNING")
        start_x = self.current_pose[0]
        start_y = self.current_pose[1]
        start_theta = self.current_pose[2]

        end_x = self.goal[0]
        end_y = self.goal[1]

        if replanning:
            print("USING INFLATED OBSTACLES")
            grid_map = self.PVC.inflate_obs(self.grid_map)
        else:
            grid_map = self.PVC.inflate_obs(self.grid_map)
            #grid_map = self.grid_map


        # Five planning trials
        for i in range(5):

            self.planner.plan(start_x, start_y, start_theta, end_x, end_y, grid_map, self.grid_origin[0], self.grid_origin[1])
            path = np.asarray(self.planner.path())
            # inverting the path to ensure it is correctly arranged
            path = path[::-1]

            if path.shape[0] > 0  and np.linalg.norm(np.asarray([end_x, end_y]) - path[-1]) < 0.4:
                break

        if path.shape[0] > 0 :

            # path post processing
            # Subsampling the path to remove excess of points
            self.path = path
            # Enabling the robot just before publishing the path
            self.robot_enabled(True)
            self.publish_path(self.path)
        else:
            self.explore_pub.publish(Bool()) # look for another frontier

    def publish_path(self, path):
        # Publish a path as a series of line markers
        if path is not None and path.shape[0] > 1:
            path_to_follow = Path()
            path_to_follow.header.frame_id = 'odom'
            path_to_follow.header.stamp = rospy.Time.now()
            path_to_follow.header.seq = 0

            for n in path:
                p = PoseStamped()
                p.pose.position.x = n[0]
                p.pose.position.y = n[1]
                p.pose.position.z = 0.0
                p.pose.orientation.w = 1
                path_to_follow.poses.append(p)
            
            self.path_pub.publish(path_to_follow)

    def robot_enabled(self, x):
        # Enable or disable robot controller
        rospy.wait_for_service('enable_control_and_wait')
        stop = rospy.ServiceProxy('enable_control_and_wait', SetBool)
        stop(x)

    def new_search(self, event):
        self.explore_pub.publish(Bool()) # look for another frontier




if __name__ == '__main__':

    print("ONLINE PATH PLANNING NODE IGNITED")

    rospy.init_node('online_planning_node')   

    # Required topics
    gridmap_topic = rospy.get_param("/octomap_topic")
    odometry_topic = rospy.get_param("/tbot_odometry_topic")

    node = OnlinePlanner(gridmap_topic, odometry_topic, 15)

    # Run forever
    rospy.spin()