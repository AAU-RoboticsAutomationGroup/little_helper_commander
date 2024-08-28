#!/usr/bin/env python3

#std python packages 
from typing import List
import numpy as np 

#ros2 packages
from rclpy.duration import Duration

from numpy.core.multiarray import array
import rclpy
import rclpy.time
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult 
from nav_msgs.msg import OccupancyGrid  
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from rclpy.time import Time

from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformListener
from tf2_ros import TransformBroadcaster 
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

import matplotlib.pyplot as plt

class Navigator(BasicNavigator):
    def __init__(self, node_name='littel_helper_navigator', namespace=''):
        super().__init__(node_name, namespace)
        
        #declaring the most important parameters for now
        self.declare_parameter("initial_pose", rclpy.Parameter.Type.DOUBLE_ARRAY) #[x, y, qw, qz]
        self.declare_parameter("item_position", rclpy.Parameter.Type.DOUBLE_ARRAY) #[x, y, z]
        self.declare_parameter("goal_pose", rclpy.Parameter.Type.DOUBLE_ARRAY) #[x, y, qw, qz]
        self.declare_parameter("behavior_tree_path", rclpy.Parameter.Type.STRING)

        self.initial_pose_l = self.get_parameter("initial_pose").value
        self.goal_pose = self.get_parameter("goal_pose").value
        self.item_position = self.get_parameter("item_position").value
        self.behavior_tree_path = self.get_parameter("behavior_tree_path").value
        self.create_subscription(Clock, 'clock', self.clock_callback, 10)
        self.create_subscription(OccupancyGrid, "global_costmap/costmap", self.costmap_callback, 10)
        
        
        self.navigation_frame_id = "map"


        self.tf_buffer = tf2_ros.Buffer() 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.parent_frame_id = "map"
        self.child_frame_id = "item"

        self.latest_tf = False
    
    def clock_callback(self, time_msg):
        """
        when a new time is recived from the clock topic, update the time variable 
        """
        self.time = Clock()
        self.time = time_msg
        self.get_logger().debug(f"{self.time.clock.sec}")

    def get_clock_time(self):
        """
        return the latest time recived from the clock topic 
        """
        try:
            time_msg = Time(seconds=self.time.clock.sec, nanoseconds=self.time.clock.nanosec) 
            return time_msg.to_msg()
        except Exception as e: 
            self.get_logger().warn(f"the clock topic probaly dont exist giving the error {e}, defaulting to using internal clock")
            return self.get_clock().now().to_msg()

    def timer_callback(self):
        """
        update the tranform of the child frame at a fixed rate 
        """
        try: 
            tf_stamped = self.tf_buffer.lookup_transform(self.child_frame_id, self.parent_frame_id, rclpy.time.Time())
            # print(f"transform recived \n ----- \n{self.cast_tf_matrix(tf_stamped.transform)} \n -----")
            self.latest_tf = tf_stamped
        except Exception as e:
            print(e)

    def get_latest_tf(self):
        """
        return the latest transform of the child frame ie the transform from the map to the item
        """
        if self.latest_tf != False:
            self.get_logger().info("returning latest tf")
            return self.latest_tf
        elif not self.latest_tf:
            self.get_logger().warn(f"no transform have been recived from the tf buffer\n defaulting to use the position specified during launch: \n{self.item_position}")
            tf = TransformStamped()
            tf.header.frame_id = self.parent_frame_id
            tf.child_frame_id = self.child_frame_id 
            tf.header.stamp = self.get_clock_time() 
            tf.transform.translation.x = self.item_position[0]
            tf.transform.translation.y = self.item_position[1]
            tf.transform.translation.z = self.item_position[2]

            return tf 



    def cast_tf_matrix(self, transform) -> np.ndarray:
        """
        cast the ros2 transform into a transformation matrix
        """
        tf_matrix = np.zeros((4,4))
        tf_matrix[3,3]=1
        tf_matrix[0,3] = transform.translation.x
        tf_matrix[1,3] = transform.translation.y
        tf_matrix[2,3] = transform.translation.z
        qx = transform.rotation.x
        qy = transform.rotation.y
        qz = transform.rotation.z
        qw = transform.rotation.w

        tf_matrix[0,0] = 1 - 2*qy**2 - 2*qz**2
        tf_matrix[0,1] = 2*qx*qy - 2*qw*qz 
        tf_matrix[0,2] = 2*qx*qz + 2*qw*qy
        tf_matrix[1,0] = 2*qx*qy + 2*qw*qz
        tf_matrix[1,1] = 1 - 2*qx**2 - 2*qz**2
        tf_matrix[1,2] = 2*qy*qz - 2*qw*qx 
        tf_matrix[2,0] = 2*qx*qz - 2*qw*qy 
        tf_matrix[2,1] = 2*qy*qz - 2*qw*qx 
        tf_matrix[2,2] = 1 - 2*qx**2 - 2*qy**2 

        return tf_matrix
    def initiate_pickup(self): 
        pass


    def costmap_callback(self, msg):
        """
        when a new costmap is recived, it is converted to a numpy array for waypoint generation 
        """
        # print(msg.info)
        self.costmap = (np.array(msg.data).reshape((msg.info.height, msg.info.width,1)))
        self.cmap_height = msg.info.height
        self.cmap_width = msg.info.width
        self.cmap_origin = msg.info.origin
        self.cmap_resolution = msg.info.resolution
        
        # adding coordinate layers to costmap, does not suport rotated costmaps 
        costmap_x_position = np.cumsum(np.ones_like(self.costmap),axis=1)*self.cmap_resolution + self.cmap_origin.position.x
        costmap_y_position = np.cumsum(np.ones_like(self.costmap),axis=0)*self.cmap_resolution + self.cmap_origin.position.y 
        # plt.imshow(np.array(msg.data).reshape((msg.info.height, msg.info.width)))
        # plt.imshow(costmap_x_position+self.costmap)
        # plt.imshow(costmap_y_position+self.costmap)
        # plt.show()

        self.costmap = np.vstack([self.costmap, costmap_x_position, costmap_y_position]).reshape((3, msg.info.height, msg.info.width)).transpose(1,2,0)
        print(self.costmap.shape)
        # plt.imshow(self.costmap[:,:,1], 'plasma')
        # plt.show()
        np.save("costmap", self.costmap)


        

    def generate_waypoints(self, initial_position : np.ndarray, goal_pose : np.ndarray, item_position : np.ndarray, costmap : np.ndarray) -> List[PoseStamped]:
        """
        function to generate waypoints, uses the costmap and the initial pose to find the best graasping waypoints 
        """
        # generate a list of trial points from a radius of the item position
        # find the point on the radius that is closest to the specified initial_position and does - 
        # and the line generated from this point does not cross the cspace 
        
        trial_resolution = 361 #the amount of points on the radius to try 

        attack_angle = 200 # the angle to the line perpendicular to the tangent of the trial value,

        radius = 2 # the radius from the item to generate trial points 

        cspace_radius = radius + 3 # the area to look for overlapping with cspace 
        
        pickup_path_length = 3 # the pickup_path_length ie how far the robot should go in a straight line 

        path_line_with = 0.2 # the with of the line used to check for overlapping with the cspace 

        points_on_radius = np.linspace(0,359,trial_resolution)

        initial_headings = points_on_radius+attack_angle

        trial_values = np.c_[points_on_radius, initial_headings]
        
  

        # plt.imshow(costmap[:,:,0])
        # plt.show()       
        # plt.imshow(costmap[:,:,1])
        # plt.show()
        # 
        # plt.imshow(costmap[:,:,2])
        # plt.show()

        selected_cspace_idx = (costmap[:,:,1] - item_position[0])**2 + (costmap[:,:,2] - item_position[1])**2 <= cspace_radius**2 
        
        # plt.imshow(selected_cspace_idx)
        # plt.show()
        self.get_logger().info(f"item_position {item_position}")
        
        print(selected_cspace_idx.shape)
        print(selected_cspace_idx[:,0].shape)

        costmap_reduced = np.zeros_like(costmap)
        costmap_reduced[selected_cspace_idx] = costmap[selected_cspace_idx]

        # costmap[selected_cspace_idx[1],selected_cspace_idx[0],:] 
               
        print(costmap_reduced.shape)
        costs = []
        line_params_list = []

        for val in trial_values:
            heading_angle = (val[1]/180)*np.pi
            angle_from_item = (val[0]/180) *np.pi
            known_intersect = np.array([np.cos(angle_from_item), np.sin(angle_from_item)])*radius + item_position 

            # ax.scatter(known_intersect[0], known_intersect[1])
            line_params =  [np.sin(heading_angle) /np.cos(heading_angle),  
                            known_intersect[1] - np.sin(heading_angle) / np.cos(heading_angle) * known_intersect[0] ] # y = ax+b, as [a, b]
            
            line_params_list.append(line_params)
            # validate the line, ie. dose it pass through the seleceted config space correctly 
            


            line_idx_1 = (costmap_reduced[:,:,2] > (costmap_reduced[:,:,1] * line_params[0] + line_params[1] - (np.sqrt(1 + line_params[0]**2)*path_line_with) ))
            line_idx_2 = (costmap_reduced[:,:,2] < (costmap_reduced[:,:,1] * line_params[0] + line_params[1] + (np.sqrt(1 + line_params[0]**2)*path_line_with) ))
            
            line_idx = line_idx_1 == line_idx_2

            
            costmap_idx = costmap_reduced[:,:,0] > 0 

            # plt.imshow(costmap_idx.astype(np.uint8) + line_idx.astype(np.uint8) == 1)
            # plt.show()

            line_config_overlap = np.sum(line_idx.astype(np.uint8) + costmap_idx.astype(np.uint8) > 1)
            
            if line_config_overlap == 0: 
                cost = np.sqrt(np.sum((initial_position-known_intersect)**2)) + line_config_overlap
                # use this plotting for debug in case os weird waypoint selection 
                # plt.imshow(line_idx.astype(np.uint8) + costmap_idx.astype(np.uint8))
                # plt.show()
                costs.append(cost)
            else:
                # plt.imshow(line_idx.astype(np.uint8) + costmap_idx.astype(np.uint8))
                # plt.show()

                costs.append(np.inf)

        print(np.array(costs))

        best_values = trial_values[np.argmin(np.array(costs))]

        heading_angle = best_values[1]/180 * np.pi 
        angle_from_item = best_values[0]/180 * np.pi

        coordinate = np.array([np.cos(angle_from_item), np.sin(angle_from_item)])*radius + item_position
        
        pre_grasp_pose = self.cast_waypoint(coordinate[0], 
                                            coordinate[1], 
                                            np.cos(heading_angle/2), 
                                            np.sin(heading_angle/2))
        

        post_grasp_coordinate = coordinate + np.array([np.cos(heading_angle), np.sin(heading_angle)])*pickup_path_length 



        post_grasp_pose = self.cast_waypoint(post_grasp_coordinate[0],
                                             post_grasp_coordinate[1],
                                             np.cos(heading_angle/2),
                                             np.sin(heading_angle/2))

        self.get_logger().info(f"generated pre and post grasp poses sucsessfully with coordinates pre: {coordinate}, post: {post_grasp_coordinate}")

        unstamped_poses = [pre_grasp_pose, post_grasp_pose]

        return unstamped_poses 
    
    def cast_waypoint(self, x, y, w, z):
        """
        helper function to construct a waypoint 
        """
        waypoint = PoseStamped()
        waypoint.header.frame_id = self.navigation_frame_id 
        waypoint.header.stamp = self.get_clock_time()
        waypoint.pose.position.x = x 
        waypoint.pose.position.y = y 
        waypoint.pose.orientation.w = w
        waypoint.pose.orientation.z = z 

        return waypoint






def main():
    rclpy.init()
    navigator = Navigator()
    navigator.waitUntilNav2Active()
    navigator.get_logger().info(f"setting the inital_pose {navigator.initial_pose_l}")
    x, y, w, z = navigator.initial_pose_l
    navigator.setInitialPose(navigator.cast_waypoint(x, y, w, z))

    item_tf = navigator.get_latest_tf()
    

    grasping_waypoints =navigator.generate_waypoints(initial_position=np.array([navigator.initial_pose_l[0], navigator.initial_pose_l[1]]),
                                                     item_position= np.array([item_tf.transform.translation.x, item_tf.transform.translation.y]),
                                                     costmap= navigator.costmap,
                                                     goal_pose= np.array([navigator.goal_pose[0], navigator.goal_pose[1]])
                                                     )

    navigator.get_logger().info(f"generated the waypoints: \n{grasping_waypoints}")
    waypoints = []

    for waypoint in grasping_waypoints:
        waypoints.append(waypoint)
    
    x, y, w, z = navigator.goal_pose
    
    goal_waypoint = navigator.cast_waypoint(x,y,z,w)
    waypoints.append(goal_waypoint)
    
    navigator.goThroughPoses(waypoints, behavior_tree=navigator.behavior_tree_path)
    
    i = 0
    while not navigator.isTaskComplete():
        # print(navigator.get_transform("map","item"))

        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption

    





    




