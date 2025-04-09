import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import numpy as np
import cv2
from image_geometry import PinholeCameraModel

bridge = CvBridge()
rgb_image = None
camera_model = None
pointcloud_pub = None

def rgb_callback(msg):
    '''
    Callback for RGB image. Converts to OpenCV and shows it
    '''
    global rgb_image
    try:
        rgb_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("RGB Image", rgb_image)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(f"Failed to convert RGB image {e}")


def camera_info_callback(msg):
    global camera_model
    try:
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(msg)
    except Exception as e:
        rospy.logerr(f"Failed to process camera info: {e}")

def publish_point_cloud(points, header):
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 12, PointField.FLOAT32, 1),
    ]

    cloud_data = []
    for x, y, z, r, g, b in points:
        rgb = (int(r) << 16) | (int(g) << 8) | int(b)
        rgb_float = np.frombuffer(np.uint32(rgb).tobytes(), dtype=np.float32)[0]
        cloud_data.append([x, y, z, rgb_float])

    cloud_msg = pc2.create_cloud(header, fields, cloud_data)
    pointcloud_pub.publish(cloud_msg)


def depth_callback(msg):
    global rgb_image, camera_model
    if camera_model is None or rgb_image is None:
        rospy.logwarn_throttle(1, "Waiting for RGB image and camera info...")
        return

    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        height, width = depth_image.shape

        fx = camera_model.fx()
        fy = camera_model.fy()
        cx = camera_model.cx()
        cy = camera_model.cy()

        points = []
        for v in range(0, height, 2):
            for u in range(0, width, 2):
                z = depth_image[v, u] / 1000.0
                if z == 0: continue

                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                color = rgb_image[v, u]
                points.append((x, y, z, color[2], color[1], color[0]))

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_color_optical_frame"

        publish_point_cloud(points, header)

    except Exception as e:
        rospy.logerr(f"Depth callback error: {e}")

if __name__ == '__main__':
    rospy.init_node("camera_listener_pointcloud", anonymous=True)

    rospy.Subscriber("/camera/color/image_raw", Image, rgb_callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)
    rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_info_callback)

    pointcloud_pub = rospy.Publisher("/point_cloud", PointCloud2, queue_size=1)

    rospy.loginfo("Camera listener and point cloud publisher started.")
    rospy.spin()