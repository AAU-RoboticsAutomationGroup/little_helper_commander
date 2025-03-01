import rclpy
from rclpy.node import Node 
from lh_interfaces.msg import PathStatus 
from std_msgs.msg import Float32 
from nav2_msgs.msg import SpeedLimit

class SpeedLimiter(Node):
    def __init__(self, node_name = "grasping_path_speed_limit"):
        super().__init__(node_name)
        self.speed_limit = SpeedLimit()
        self.speed_limit.speed_limit = 0.1

        self.higher_speed_limit = SpeedLimit()
        self.higher_speed_limit.speed_limit = 9.99

        self.create_subscription(PathStatus, "grasping_path_status", self.grasping_path_status_callback, 10 )
        self.speed_limit_publisher = self.create_publisher(SpeedLimit, "speed_limit", 10)

    def grasping_path_status_callback(self, msg : PathStatus):
        if msg.trigger: 
            self.speed_limit_publisher.publish(self.speed_limit)
        else:
            self.speed_limit_publisher.publish(self.higher_speed_limit)

def main():
    rclpy.init()
    speed_limit_node = SpeedLimiter()
    rclpy.spin(speed_limit_node)
    rclpy.shutdown()
