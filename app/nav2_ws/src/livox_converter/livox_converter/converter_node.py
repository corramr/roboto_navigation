import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import struct

class LivoxCustomToPC2(Node):
    def __init__(self):
        super().__init__('livox_custom_to_pc2_node')
        
        # Subscribe to Custom format
        self.subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',
            self.listener_callback,
            10)
            
        # Publish PointCloud2 format
        self.publisher = self.create_publisher(PointCloud2, '/livox/lidar_pc2', 10)
        self.get_logger().info('Livox Custom -> PC2 Converter Node Started')

    def listener_callback(self, msg: CustomMsg):
        pc2_msg = PointCloud2()
        pc2_msg.header = msg.header
        pc2_msg.height = 1
        pc2_msg.width = msg.point_num
        
        pc2_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 16 
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width
        pc2_msg.is_dense = True

        buffer = []
        for point in msg.points:
            buffer.append(struct.pack('ffff', point.x, point.y, point.z, float(point.reflectivity)))

        pc2_msg.data = b''.join(buffer)
        self.publisher.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LivoxCustomToPC2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()