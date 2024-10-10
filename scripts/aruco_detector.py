import cv2
import cv2.aruco as aruco
import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from enshu2.msg import Marker, MarkerArray  # Import custom messages
from cv_bridge import CvBridge, CvBridgeError

class ArucoMarkerPublisher:
    def __init__(self):
        rospy.init_node('aruco_marker_publisher', anonymous=True)
        
        # Subscribe to camera info and compressed image topics
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.image_callback)
        self.marker_pub = rospy.Publisher('/aruco_marker_publisher/markers', MarkerArray, queue_size=10)
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)  # Using Aruco markers with 6x6 grid and 250 ids
        self.aruco_params = aruco.DetectorParameters_create()

    def camera_info_callback(self, camera_info):
        # Get camera intrinsic parameters from the CameraInfo message
        self.camera_matrix = np.array(camera_info.K).reshape((3, 3))
        self.dist_coeffs = np.array(camera_info.D)

    def image_callback(self, data):
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Camera info not yet received")
            return

        try:
            # Convert the compressed image to OpenCV format
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            rospy.logerr(f"Could not decode compressed image: {str(e)}")
            return

        # Resize the image to make detection faster
        scale_factor = 1.0  # Adjust this factor to balance speed vs accuracy
        resized_image = cv2.resize(cv_image, None, fx=scale_factor, fy=scale_factor)

        # Scale the camera matrix to match the resized image
        scaled_camera_matrix = self.camera_matrix.copy()
        scaled_camera_matrix[0, 0] *= scale_factor  # fx
        scaled_camera_matrix[1, 1] *= scale_factor  # fy
        scaled_camera_matrix[0, 2] *= scale_factor  # cx
        scaled_camera_matrix[1, 2] *= scale_factor  # cy

        # Detect Aruco markers in the resized image
        corners, ids, _ = aruco.detectMarkers(resized_image, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            marker_array = MarkerArray()

            for i, marker_id in enumerate(ids):
                # Scale the corners back to the original image size
                marker_corners = corners[i].reshape((4, 2)) / scale_factor  # Scale back

                # Estimate the pose of the marker using solvePnP with the scaled camera matrix
                marker_size = 0.048  # Adjust according to your marker size
                object_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                        [marker_size / 2, marker_size / 2, 0],
                                        [marker_size / 2, -marker_size / 2, 0],
                                        [-marker_size / 2, -marker_size / 2, 0]])

                # Use the scaled camera matrix in solvePnP
                retval, rvec, tvec = cv2.solvePnP(object_points, marker_corners, scaled_camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
            
                # Convert rotation vector to quaternion (for pose orientation)
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

                # Create marker pose
                marker_pose = Pose()
                marker_pose.position = Point(tvec[0][0], tvec[1][0], tvec[2][0])
                marker_pose.orientation = Quaternion(*quaternion)

                # Create a Marker message
                marker_msg = Marker()
                marker_msg.header = Header()
                marker_msg.header.stamp = rospy.Time.now()
                marker_msg.header.frame_id = "camera_color_optical_frame"
                marker_msg.id = int(marker_id[0])  # Marker ID
                marker_msg.pose = marker_pose

                # Save the corners in the marker message (for drawing the box)
                for corner in marker_corners:
                    corner_point = Point()
                    corner_point.x = corner[0]
                    corner_point.y = corner[1]
                    corner_point.z = 0  # 2D image, so z = 0
                    marker_msg.corners.append(corner_point)

                # Add the marker to the array
                marker_array.markers.append(marker_msg)

            # Publish the detected markers
            self.marker_pub.publish(marker_array)

        # Draw detected markers for visualization (optional)
        #cv_image_with_markers = aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)
        #cv2.imshow("Aruco Markers", cv_image_with_markers)
        #cv2.waitKey(1)


    def rotation_matrix_to_quaternion(self, R):
        """
        Convert a rotation matrix to a quaternion.
        """
        q_w = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        q_x = (R[2, 1] - R[1, 2]) / (4.0 * q_w)
        q_y = (R[0, 2] - R[2, 0]) / (4.0 * q_w)
        q_z = (R[1, 0] - R[0, 1]) / (4.0 * q_w)
        return [q_x, q_y, q_z, q_w]

if __name__ == '__main__':
    try:
        ArucoMarkerPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
