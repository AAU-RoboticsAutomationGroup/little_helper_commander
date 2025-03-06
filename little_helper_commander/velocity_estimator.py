import rclpy
import rclpy.time
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, Time
from tf2_ros import TransformListener
from tf2_ros import TransformBroadcaster 
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TwistStamped 
import numpy as np

def get_yaw(tf):
    qz = tf.transform.rotation.z
    qw = tf.transform.rotation.w 
    qy = tf.transform.rotation.y
    qx = tf.transform.rotation.x
    aa = 2 * (qw * qz + qx * qy)  
    bb = 1 - 2 * (qy * qy + qz * qz) 
    yaw = np.arctan2(aa, bb) 

    return yaw 


class VelocityEstimator(Node):
    def __init__(self, node_name="velocity_estimator"):
        super().__init__(node_name)
        self.child_frame_id = "item"
        self.parent_frame_id = "ur5_base_link"
        
        self.tf_buffer = tf2_ros.Buffer() 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.vel_pub = self.create_publisher(TwistStamped, "vel_arm_base", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.base_tf_list = []
        
    def timer_callback(self):
        try:
            tf_stamped = self.tf_buffer.lookup_transform(self.parent_frame_id, self.child_frame_id, rclpy.time.Time())
        # except Exception as e:
        #     self.get_logger().warn(f'1 {e}')     
        # try:
            if len(self.base_tf_list) < 50:
                self.base_tf_list.append(tf_stamped)
            else:
                init_time = Time.from_msg(self.base_tf_list[0].header.stamp).nanoseconds
                # self.get_logger().info(f"init time {init_time* 1e-9}")
                # self.get_logger().info(f"tf queue length {len(self.base_tf_list)}")
                init_x = self.base_tf_list[0].transform.translation.x 
                init_y = self.base_tf_list[0].transform.translation.y 
                init_pos = np.array([init_x, init_y])

                init_yaw = get_yaw(self.base_tf_list[0])

                # self.get_logger().info(f"init pos {init_x}, {init_y}")
                
                self.base_tf_list.pop(0)
                self.base_tf_list.append(tf_stamped)
                
                vel = np.array([0.0,0.0])
                yaw_vel = 0.0
                n = 0
                for i, tf in enumerate(self.base_tf_list):
                    time = Time.from_msg(tf.header.stamp).nanoseconds
                    d_time = (time - init_time) * 1e-9
                    # self.get_logger().info(f"d time {d_time}")
                    # self.get_logger().info(f"current time {time * 1e-9}")
                    x = tf.transform.translation.x 
                    y = tf.transform.translation.y
                    pos = np.array([x, y])
                    
                    yaw = get_yaw(tf)

                    d_yaw = yaw - init_yaw

                    # self.get_logger().info(f"pos {pos}")
                    d_pos = pos - init_pos
                    # self.get_logger().info(f"d pos {d_pos}")
                    # self.get_logger().info(f"d yaw {d_yaw}")

                    try:
                        vel += d_pos / d_time 
                        yaw_vel += d_yaw / d_time
                        n += 1
                    except Exception as e:
                        self.get_logger().debug(f'{e}')

                vel = vel/n 
                yaw_vel = yaw_vel/n 

                # self.get_logger().info(f"vel {vel}")

                vel_msg = TwistStamped()

                vel_msg.header.frame_id = self.parent_frame_id
                # vel_msg.header.stamp = self.get_clock().now().to_msg()

                vel_msg.twist.linear.x = vel[0]
                vel_msg.twist.linear.y = vel[1]

                vel_msg.twist.angular.z = yaw_vel



                self.vel_pub.publish(vel_msg)
        except Exception as e:
            self.get_logger().debug(f'2 {e}')

def main():
    rclpy.init()
    velocity_estimator = VelocityEstimator()
    rclpy.spin(velocity_estimator)
    rclpy.shutdown()





                
        

