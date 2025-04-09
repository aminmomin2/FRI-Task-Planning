import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import struct

class PointCloudTransformer:
    def __init__(self):
        # Initialize the node
        rospy.init_node('point_cloud_transformer', anonymous=True)
        
        # Setup TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Setup subscribers and publishers
        self.cloud_sub = rospy.Subscriber(
            "/point_cloud",  # Subscribe to original point cloud
            PointCloud2,
            self.point_cloud_callback
        )
        
        self.transformed_cloud_pub = rospy.Publisher(
            "/transformed_point_cloud",  # Publish transformed point cloud
            PointCloud2,
            queue_size=1
        )
        
        rospy.loginfo("Point cloud transformer node started")

    def point_cloud_callback(self, cloud_msg):
        try:
            # Get transform from camera frame to robot frame
            transform = self.tf_buffer.lookup_transform(
                "base_link",  # target frame (robot)
                cloud_msg.header.frame_id,  # source frame (camera)
                cloud_msg.header.stamp,
                rospy.Duration(1.0)
            )
            
            # Convert PointCloud2 to list of points
            points = []
            for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z", "rgb")):
                x, y, z = p[0:3]
                rgb = p[3]
                
                # Extract RGB values correctly
                rgb_bytes = struct.pack('I', int(rgb))  # Pack as unsigned int
                r = rgb_bytes[0]
                g = rgb_bytes[1]
                b = rgb_bytes[2]
                
                points.append((x, y, z, r, g, b))
            
            # Transform points
            transformed_points = []
            for x, y, z, r, g, b in points:
                point = PointStamped()
                point.header = cloud_msg.header
                point.point.x = x
                point.point.y = y
                point.point.z = z
                
                transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)
                transformed_points.append((
                    transformed_point.point.x,
                    transformed_point.point.y,
                    transformed_point.point.z,
                    r, g, b
                ))
            
            # Create new point cloud message
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.FLOAT32, 1),
            ]

            cloud_data = []
            for x, y, z, r, g, b in transformed_points:
                rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                rgb_float = np.frombuffer(np.uint32(rgb).tobytes(), dtype=np.float32)[0]
                cloud_data.append([x, y, z, rgb_float])

            # Create and publish transformed cloud
            header = cloud_msg.header
            header.frame_id = "base_link"
            transformed_cloud = pc2.create_cloud(header, fields, cloud_data)
            self.transformed_cloud_pub.publish(transformed_cloud)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform failed: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")

if __name__ == '__main__':
    try:
        transformer = PointCloudTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass