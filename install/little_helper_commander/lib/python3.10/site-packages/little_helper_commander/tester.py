from little_helper_commander import Navigator, TF2Interface
from tf2_ros import StaticTransformBroadcaster 
import spatialmath as sp 
import numpy as np 
print(Navigator)
spqt = sp.Quaternion()
from geometry_msgs.msg import TransformStamped
import rclpy
import time 




if __name__ == "__main__":
    rclpy.init()
    navigator = Navigator("test_navigator")
    tf_test_transform_publisher = StaticTransformBroadcaster(navigator)
    
    tf_tester = TF2Interface()

    test_tf = TransformStamped() 
    test_tf.transform.translation.x = 1.1
    test_tf.transform.translation.y = 2.2 
    test_tf.transform.translation.z = 3.3 
    test_tf.transform.rotation.z = -0.707 
    test_tf.transform.rotation.w = 0.707
    test_tf.child_frame_id = "test_child"
    test_tf.header.frame_id = "test_root"

    while 1:
        tf_test_transform_publisher.sendTransform(test_tf)
        print(np.round(tf_tester.cast_tf_matrix(test_tf.transform), 3))
        time.sleep(1)

    
