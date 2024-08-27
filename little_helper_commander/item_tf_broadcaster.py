import rclpy
import rclpy.time
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformListener
from tf2_ros import TransformBroadcaster 
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped



class ItemTFBroadcaster(Node):
    """
    node to maintain a tf from the map to the item.
    if another node publishes this transfom, this node should update the transform being published

    """
    def __init__(self, node_name="item_tf_broadcaster"):
        super().__init__(node_name)
        self.item_tf = TransformStamped()
        self.item_tf.child_frame_id = "item"
        self.item_tf.header.frame_id = "map"
        self.declare_parameter('item_position', rclpy.Parameter.Type.DOUBLE_ARRAY)
        item_initial_position = self.get_parameter('item_position').value
        self.item_tf.transform.translation.x = item_initial_position[0]
        self.item_tf.transform.translation.y = item_initial_position[1]
        self.item_tf.transform.translation.z = item_initial_position[2]

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer() 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.parent_frame_id = "map"
        self.child_frame_id = "item"


        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try: 
            tf_stamped = self.tf_buffer.lookup_transform(self.child_frame_id, self.parent_frame_id, rclpy.time.Time())
            # print(f"transform recived \n ----- \n{self.cast_tf_matrix(tf_stamped.transform)} \n -----")
            self.item_tf = tf_stamped
        except Exception as e:
            print(e)

        self.item_tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.item_tf)
    
        pass

def main():
    rclpy.init()
    item_tf_broadcaster = ItemTFBroadcaster()
    rclpy.spin(item_tf_broadcaster)
    rclpy.shutdown()


