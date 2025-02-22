#!/usr/bin/env python3

#std python packages 
from locale import Error
from logging import raiseExceptions
from typing import FrozenSet, List, cast

from sympy.geometry import point
from action_msgs.msg import GoalStatus
import numpy as np 
import time 
import ast 

#ros2 packages
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration

from numpy.core.multiarray import array
import rclpy
import rclpy.time
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult 
from nav_msgs.msg import OccupancyGrid  
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovariance,  PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from nav2_msgs.action import ComputePathThroughPoses, SmoothPath 
from nav_msgs.msg import Path 
import std_msgs.msg

from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformListener
from tf2_ros import TransformBroadcaster 
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from lh_interfaces.msg import PathStatus





try:
    import matplotlib.pyplot as plt
except Exception as e:
    print(e)





class GraspingNavigator(Node):
    def __init__(self, node_name='littel_helper_grasping_navigator'):
        super().__init__(node_name) 

        self.basic_navigator = BasicNavigator() 

        #declaring the most important parameters for now
        # self.declare_parameter("initial_pose", rclpy.Parameter.Type.DOUBLE_ARRAY) #[x, y, qw, qz]
        # self.declare_parameter("item_position", rclpy.Parameter.Type.DOUBLE_ARRAY) #[x, y, z]
        # self.declare_parameter("goal_pose", rclpy.Parameter.Type.DOUBLE_ARRAY) #[x, y, qw, qz]
        self.declare_parameter("behavior_tree_path", rclpy.Parameter.Type.STRING)

        # self.initial_pose_l = self.get_parameter("initial_pose").value
        # self.goal_pose = self.get_parameter("goal_pose").value
        # self.item_position = self.get_parameter("item_position").value
        self.behavior_tree_path = self.get_parameter("behavior_tree_path").value
        self.create_subscription(Clock, 'clock', self.clock_callback, 10)
        self.create_subscription(OccupancyGrid, "global_costmap/costmap", self.costmap_callback, 10)
        
        self.create_subscription(PoseStamped, "grasping_goal_pose", self.goal_pose_callback, 10)
        self.create_subscription(PointStamped, "item_position", self.item_position_callback, 10)
        
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.amcl_callback, 10)

        self.create_subscription(std_msgs.msg.String, "webui_waypoints", self.webui_weypoints_callback, 10)
        self.create_subscription(std_msgs.msg.Bool, "webui_move_trigger", self.webui_move_callback ,10)

        self.amcl_pose = PoseWithCovarianceStamped()
        self.initial_pose = PoseStamped()
        self.amcl_recived = False 


        self.item_position = np.ndarray(2)
        self.item_recived = False
        self.goal_pose = np.ndarray(4)
        

        self.grasping_path_publisher = self.create_publisher(Path, 'grasping_path', 10)

        self.navigation_frame_id = "map"


        self.tf_buffer = tf2_ros.Buffer() 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.parent_frame_id = "map"
        self.child_frame_id = "item"


        # self.robot_base_frame_id = "little_helper/chassis"

        self.robot_base_frame_id = "base_link"
        
        self.pre_grasp_reached = False
        self.post_grasp_reached = False

        self.pre_grasp_reached_state_publisher = self.create_publisher(std_msgs.msg.Bool, "pre_grasp_state", 10)
        self.post_grasp_reached_state_publisher = self.create_publisher(std_msgs.msg.Bool, "post_grasp_state", 10)

        self.grasping_path_state_publisher = self.create_publisher(PathStatus, "grasping_path_status", 10)

        self.grasping_waypoints = False

        self.latest_tf = False
    
        self.costmap = np.array([False])
        self.costmap_recived = False
        
        self.generated_path = None
        self.have_path_been_generated = False

        self.basic_navigator.waitUntilNav2Active()

        self.waypoints = []
        self.path = []
        self.grasping_paths = []


    def webui_weypoints_callback(self, webui_waypoints):
        self.get_logger().info(f"recived waypoints from the web ui {webui_waypoints.data}")

        data_list = ast.literal_eval(webui_waypoints.data)
        self.waypoints = []
        annotated_waypoints = [] 
        for data_point in data_list:
            point = PoseStamped()
            self.get_logger().info(f"datapoint {data_point}")
            point.pose.position.x = float(data_point["xm"])
            point.pose.position.y = float(data_point["ym"])

            if len(self.waypoints) == 0: 
                pre_wp = self.initial_pose
                
            else:
                pre_wp = self.waypoints[-1]

            current_wp = point 
            vector_from_prev_wp = np.array([current_wp.pose.position.x - pre_wp.pose.position.x,
                                            current_wp.pose.position.y - pre_wp.pose.position.y])
            
            heading_vec = vector_from_prev_wp/np.linalg.norm(vector_from_prev_wp)

            heading_angle = np.arctan2(heading_vec[1], heading_vec[0])

            qw = np.cos(heading_angle/2)
            qz = np.sin(heading_angle/2)


            waypoint = self.cast_waypoint(data_point["xm"], data_point["ym"], qw, qz)

            if data_point['type'] == "start":
                self.waypoints.append(waypoint)
                annotated_waypoint = {'type':data_point['type'], 'waypoint':waypoint}
                annotated_waypoints.append(annotated_waypoint)

            elif data_point['type'] == "end":
                self.waypoints.append(waypoint)
                annotated_waypoint = {'type':data_point['type'], 'waypoint':waypoint}
                annotated_waypoints.append(annotated_waypoint)

            elif data_point['type'] == "collect":
                if len(self.waypoints) == 0:
                    pre_wp = self.initial_pose
                else:
                    pre_wp = self.waypoints[-1] 
                
                pre_pos = [pre_wp.pose.position.x, pre_wp.pose.position.y]
                item_pos = [point.pose.position.x, point.pose.position.y]

                grasping_waypoints = self.generate_waypoints_from_coordinates(pre_pos, item_pos)
                self.waypoints += grasping_waypoints
                for wp, part in zip(grasping_waypoints, ['start', 'end']):
                    annotated_waypoint = {'type':data_point['type']+part, 'waypoint':wp}
                    annotated_waypoints.append(annotated_waypoint)


            elif data_point['type'] == "dropoff":
                pre_wp = self.waypoints[-1] 
                
                pre_pos = [pre_wp.pose.position.x, pre_wp.pose.position.y]
                item_pos = [point.pose.position.x, point.pose.position.y]

                grasping_waypoints = self.generate_waypoints_from_coordinates(pre_pos, item_pos)
                
                for wp, part in zip(grasping_waypoints, ['start', 'end']):
                    annotated_waypoint = {'type':data_point['type']+part, 'waypoint':wp}
                    annotated_waypoints.append(annotated_waypoint)
                
                self.waypoints += grasping_waypoints
        
        path = self.basic_navigator.getPathThroughPoses(self.initial_pose, self.waypoints)
        
        if not path:
            self.get_logger().warn("no path genrated")
            return None

        path = self.basic_navigator.smoothPath(path)
        
        self.path = [path]
        
        for i, annot_wp in enumerate(annotated_waypoints):
            wp = annot_wp['waypoint']
            if annot_wp['type'] == "collectstart" or annot_wp['type'] == "dropoffstart":
            
                grasping_path = self.snip_grasping_path(self.waypoints[i], self.waypoints[i+1], path)

                self.grasping_path = grasping_path

                self.grasping_path_publisher.publish(grasping_path)

                self.grasping_paths.append(grasping_path)


    def webui_move_callback(self, msg):
        self.get_logger().info(f"recived move trigger signal from webui {msg.data}")
        if msg.data and len(self.path) != 0:
            # creates a timer callback 
            self.initiate_path_position_state_publisher()
            self.basic_navigator.followPath(self.path[-1])
            



    def amcl_callback(self, amcl_msg:PoseWithCovarianceStamped):
        self.amcl_pose = amcl_msg
        self.amcl_recived = True
        self.initial_pose.pose.position = self.amcl_pose.pose.pose.position
        self.initial_pose.pose.orientation = self.amcl_pose.pose.pose.orientation
        

        self.get_logger().info(f"amcl pose recieved, {self.amcl_pose.pose.pose.position.x}, {self.amcl_pose.pose.pose.position.y} ")


    def plan_callback(self, plan_msg):
        self.get_logger().info("plan recieved")
        self.path = plan_msg
    

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
            self.get_logger().warn(f"the clock topic probaly dont exist giving the error {e}, defaulting to using zero as time")
            return rclpy.time.Time().to_msg()

    def goal_pose_callback(self, goal_msg):
        """
        callback for when goal pose recived, updates state value, and initializes the pickup
        """

        self.get_logger().info("goal pose recieved")

        self.goal_pose = self.cast_waypoint(goal_msg.pose.position.x,
                                            goal_msg.pose.position.y,
                                            goal_msg.pose.orientation.w,
                                            goal_msg.pose.orientation.z)

        if (not self.costmap_recived and not self.item_recived):
            print(f"costmap or item position missing, \ncostmap: \n{self.costmap} \nitem_position: \n{self.item_position}")
            return 

        self.grasping_waypoints = self.generate_waypoints()

        waypoints=[]

        for waypoint in self.grasping_waypoints:
            waypoints.append(waypoint)

        waypoints.append(self.goal_pose)

        self.get_logger().info("waypoints generated")
               
        # initial_pose = self.initial_pose 
        # self.get_logger().info(f"initial pose: {self.initial_pose.pose.position.x} {self.initial_pose.pose.position.y} \nwaypoints: {waypoints}")

        path = self.basic_navigator.getPathThroughPoses(self.initial_pose, waypoints)

        path = self.basic_navigator.smoothPath(path)

        grasping_path = self.snip_grasping_path(waypoints[0], waypoints[1], path)

        self.grasping_path = grasping_path

        self.grasping_path_publisher.publish(grasping_path)
        
        # creates a timer callback 
        self.initiate_path_position_state_publisher()

        self.pre_grasp_reached = False
        self.post_grasp_reached = False

        self.get_logger().info("path generated !!!")

        self.basic_navigator.followPath(path)

    
    def item_position_callback(self, item_msg):
        """
        callback for when item position recived, updates state value
        """
        self.item_position[0] = item_msg.point.x
        self.item_position[1] = item_msg.point.y
        # self.item_position[2] = item_msg.point.z 
        
        self.item_recived = True 



    def timer_callback(self):
        """
        update the tranform of the child frame at a fixed rate 
        """
        try: 
            tf_stamped = self.tf_buffer.lookup_transform(self.parent_frame_id, self.child_frame_id, rclpy.time.Time())
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
        # print(self.costmap.shape)
        self.get_logger().info("cost map recived")
        # plt.imshow(self.costmap[:,:,1], 'plasma')
        # plt.show()
        np.save("costmap", self.costmap)
        
        self.costmap_recived = True

        
    def generate_waypoints_from_coordinates(self, pre_position, item_position) -> List[PoseStamped]:
        """
        function to generate waypoints, uses the costmap, the pre pose, and item position to find the best grasping waypoints 
        """
        # generate a list of trial points from a radius of the item position
        # find the point on the radius that is closest to the specified initial_position and does - 
        # and the line generated from this point does not cross the cspace 
        
        costmap = self.costmap
        initial_position = pre_position 

        trial_resolution = 50 #the amount of points on the radius to try 
        distance_to_center = 0.6
        radius = 1.5 # the radius from the item to generate trial points 
        attack_angle = np.arcsin(distance_to_center/radius)*180/np.pi + 180 # the angle to the line perpendicular to the tangent of the trial value,
        self.get_logger().info(f"attack angle = {attack_angle}")
        cspace_radius = radius + 0.1 # the area to look for overlapping with cspace 
        pickup_path_length = radius*1.3 # the pickup_path_length ie how far the robot should go in a straight line 
        path_line_with = 0.05 # the with of the line used to check for overlapping with the cspace 
        points_on_radius = np.linspace(0,359,trial_resolution)
        initial_headings = points_on_radius+attack_angle
        trial_values = np.c_[points_on_radius, initial_headings] 
        line_overlap_threshold = 50
        costmap_threshold = 80

        selected_cspace_idx = (costmap[:,:,1] - item_position[0])**2 + (costmap[:,:,2] - item_position[1])**2 <= cspace_radius**2 
        self.get_logger().info(f"item_position {item_position}")

        costmap_reduced = np.zeros_like(costmap)
        costmap_reduced[selected_cspace_idx] = costmap[selected_cspace_idx]
        costs = []
        line_params_list = []

        for val in trial_values:
            heading_angle = (val[1]/180)*np.pi
            angle_from_item = (val[0]/180) *np.pi
            known_intersect = np.array([np.cos(angle_from_item), np.sin(angle_from_item)])*radius + item_position 

            line_params =  [np.sin(heading_angle) /np.cos(heading_angle),  
                            known_intersect[1] - np.sin(heading_angle) / np.cos(heading_angle) * known_intersect[0] ] # y = ax+b, as [a, b]
            line_params_list.append(line_params)

            line_idx_1 = (costmap_reduced[:,:,2] > (costmap_reduced[:,:,1] * line_params[0] + line_params[1] - (np.sqrt(1 + line_params[0]**2)*path_line_with) ))
            line_idx_2 = (costmap_reduced[:,:,2] < (costmap_reduced[:,:,1] * line_params[0] + line_params[1] + (np.sqrt(1 + line_params[0]**2)*path_line_with) ))
            
            line_idx = line_idx_1 == line_idx_2
            
            costmap_idx = costmap_reduced[:,:,0] > costmap_threshold 

            line_config_overlap = np.sum(line_idx.astype(np.uint8) + costmap_idx.astype(np.uint8) > 1)
            
            if line_config_overlap < line_overlap_threshold: #allowing for some overlapping to make it more stable 
                cost = np.sqrt(np.sum((initial_position-known_intersect)**2)) + line_config_overlap
                costs.append(cost)
            else:
                costs.append(np.inf)
            
        if (np.array(costs)==np.inf).all():
            raise Error("no viable grasping paths found :(")
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

    def generate_waypoints(self) -> List[PoseStamped]:
        """
        function to generate waypoints, uses the costmap and the initial pose to find the best graasping waypoints 
        """
        # generate a list of trial points from a radius of the item position
        # find the point on the radius that is closest to the specified initial_position and does - 
        # and the line generated from this point does not cross the cspace 
        
        costmap = self.costmap
        item_position = self.item_position
        initial_position = np.array([self.initial_pose.pose.position.x, self.initial_pose.pose.position.y])

        trial_resolution = 50 #the amount of points on the radius to try 
        distance_to_center = 0.6
        radius = 1.5 # the radius from the item to generate trial points 
        attack_angle = np.arcsin(distance_to_center/radius)*180/np.pi + 180 # the angle to the line perpendicular to the tangent of the trial value,
        self.get_logger().info(f"attack angle = {attack_angle}")
        cspace_radius = radius + 0.1 # the area to look for overlapping with cspace 
        pickup_path_length = radius*1.3 # the pickup_path_length ie how far the robot should go in a straight line 
        path_line_with = 0.05 # the with of the line used to check for overlapping with the cspace 
        points_on_radius = np.linspace(0,359,trial_resolution)
        initial_headings = points_on_radius+attack_angle
        trial_values = np.c_[points_on_radius, initial_headings] 
        line_overlap_threshold = 50
        costmap_threshold = 80
        selected_cspace_idx = (costmap[:,:,1] - item_position[0])**2 + (costmap[:,:,2] - item_position[1])**2 <= cspace_radius**2 
        
        # plt.imshow(selected_cspace_idx)
        # plt.show()
        self.get_logger().info(f"item_position {item_position}")
        
        # print(selected_cspace_idx.shape)
        # print(selected_cspace_idx[:,0].shape)

        costmap_reduced = np.zeros_like(costmap)
        costmap_reduced[selected_cspace_idx] = costmap[selected_cspace_idx]

               
        # print(costmap_reduced.shape)
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

            
            costmap_idx = costmap_reduced[:,:,0] > costmap_threshold 

            # plt.imshow(costmap_idx.astype(np.uint8) + line_idx.astype(np.uint8) == 1)
            # plt.show()

            line_config_overlap = np.sum(line_idx.astype(np.uint8) + costmap_idx.astype(np.uint8) > 1)
            
            if line_config_overlap < line_overlap_threshold: #allowing for some overlapping to make it more stable 
                cost = np.sqrt(np.sum((initial_position-known_intersect)**2)) + line_config_overlap
                # use this plotting for debug in case os weird waypoint selection 
                # plt.imshow(line_idx.astype(np.uint8) + costmap_idx.astype(np.uint8))
                # plt.title(f"overlap with range {line_config_overlap}")
                # plt.show()
                costs.append(cost)
            else:
                # plt.imshow(line_idx.astype(np.uint8) + costmap_idx.astype(np.uint8), cmap="plasma")
                # plt.title(f"too much overlap {line_config_overlap}")
                # plt.show()
                #
                costs.append(np.inf)
            
        if (np.array(costs)==np.inf).all():
            raise Error("no viable grasping paths found :(")


        # print(np.array(costs))

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
        waypoint.pose.position.x = float(x) 
        waypoint.pose.position.y = float(y) 
        waypoint.pose.orientation.w = float(w)
        waypoint.pose.orientation.z = float(z) 

        return waypoint




    def snip_grasping_path(self, pre_grasp_waypoint, post_grasp_waypoint, path):
        def distance(a, b):
            return np.sum(np.sqrt((a-b)**2))
        pre_grasp_coor = np.array([pre_grasp_waypoint.pose.position.x, pre_grasp_waypoint.pose.position.y])

        post_grasp_coor = np.array([post_grasp_waypoint.pose.position.x, post_grasp_waypoint.pose.position.y])

        pre_grasp_reached = False 
        post_grasp_reaced = False
        
        return_path = Path() 
        return_path.header = path.header

        self.get_logger().info(f"{path.poses}")

        for pose in path.poses:
            path_point_coor = np.array([pose.pose.position.x, pose.pose.position.y])
            distance_to_pre_grasp = distance(path_point_coor, pre_grasp_coor)
            distance_to_post_grasp = distance(path_point_coor, post_grasp_coor)
            self.get_logger().info(f"distanses pre: {distance_to_pre_grasp}, post: {distance_to_post_grasp}")
            if distance_to_pre_grasp < 0.1:
                pre_grasp_reached = True 
            if distance_to_post_grasp < 0.1:
                post_grasp_reaced = True

            if pre_grasp_reached and not post_grasp_reaced:
                return_path.poses.append(pose)
                self.get_logger().info(f"appending to grasping_path")
         
        return return_path 
    
    def initiate_path_position_state_publisher(self):
        self.robot_state_time = self.create_timer(0.1, self.robot_path_state_timer_callback)

    def find_path_index(self, path, position):

        x_arr = []
        y_arr = []
        for pose in path.poses:
    
            x_arr.append(pose.pose.position.x)
            y_arr.append(pose.pose.position.y)

        x_arr = np.array(x_arr)
        y_arr = np.array(y_arr)

        pos_x = position[0]
        pos_y = position[1]
        
        dist_arr = np.sqrt((x_arr - pos_x)**2 + (y_arr - pos_y)**2)


        if dist_arr.shape[0] == 0:
            return 0
        else:
            self.get_logger().info(f"computed the distances to all the path indecies {dist_arr}")

            return int(np.argmin(dist_arr))


        
    def robot_path_state_timer_callback(self):
        def distance(a, b):
            return np.sum(np.sqrt((a-b)**2))
        
        grasping_waypoints = self.grasping_waypoints
        path_status_msg = PathStatus()    
        

        try:
            robot_position = self.tf_buffer.lookup_transform(self.parent_frame_id, self.robot_base_frame_id, rclpy.time.Time())
            
            robot_coor = np.array([robot_position.transform.translation.x, robot_position.transform.translation.y])
            pre_grasp_coor = np.array([grasping_waypoints[0].pose.position.x, grasping_waypoints[0].pose.position.y])
            post_grasp_coor = np.array([grasping_waypoints[1].pose.position.x, grasping_waypoints[1].pose.position.y])

            distance_to_pre_grasp = distance(robot_coor, pre_grasp_coor)
            distance_to_post_grasp = distance(robot_coor, post_grasp_coor)
            if distance_to_pre_grasp < 0.2:
                self.pre_grasp_reached = True 
                self.post_grasp_reached = False
            if distance_to_post_grasp < 0.3:
                self.post_grasp_reaced = True
                self.pre_grasp_reached = False
            
            self.get_logger().info(f"{distance_to_pre_grasp}")
            pre_grasp_msg = std_msgs.msg.Bool()
            pre_grasp_msg.data = self.pre_grasp_reached

            post_grasp_msg = std_msgs.msg.Bool()
            post_grasp_msg.data = self.post_grasp_reached

            self.pre_grasp_reached_state_publisher.publish(pre_grasp_msg)
            self.post_grasp_reached_state_publisher.publish(post_grasp_msg)

            
            path_status_msg.current_path_index = self.find_path_index(self.grasping_path, robot_coor)
            path_status_msg.trigger = self.pre_grasp_reached and not self.post_grasp_reached 
            path_status_msg.header.stamp = self.get_clock().now().to_msg()

            self.grasping_path_state_publisher.publish(path_status_msg)


        except Exception as e: 
            self.get_logger().warn(e)



def main():
    rclpy.init()
    navigator = GraspingNavigator()
    rclpy.spin(navigator)
