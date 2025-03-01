import rclpy
from rclpy.node import Node 
from lh_interfaces.msg import PathStatus 
from nav2_msgs.msg import SpeedLimit

class SpeedLimiter(Node):
    def __init__(self, node_name = "grasping_path_speed_limit"):
        super().__init__(node_name)
        self.speed_limit = SpeedLimit()
        self.speed_limit.percentage = False   
        self.speed_limit.speed_limit = 0.1
        
        self.higher_speed_limit = SpeedLimit()
        self.higher_speed_limit.percentage = True  
        self.higher_speed_limit.speed_limit = 100.0

        self.create_subscription(PathStatus, "grasping_path_status", self.grasping_path_status_callback, 10 )
        self.speed_limit_publisher = self.create_publisher(SpeedLimit, "/speed_limit", 10)

    def grasping_path_status_callback(self, msg : PathStatus):
        self.get_logger().info("grasping path status recieved")
        if msg.trigger: 
            self.speed_limit_publisher.publish(self.speed_limit)
            self.get_logger().info(f'Published Speed Limit: {self.speed_limit.speed_limit} m/s')

        else:
            self.speed_limit_publisher.publish(self.higher_speed_limit)
            self.get_logger().info(f'Published Speed Limit: {self.higher_speed_limit.speed_limit} m/s')


def main():
    rclpy.init()
    speed_limit_node = SpeedLimiter()
    rclpy.spin(speed_limit_node)
    speed_limit_node.destroy_node()
    rclpy.shutdown()
