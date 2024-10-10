import rospy
import tf
from math import sqrt, atan2
from enshu2.msg import MarkerArray,MarkerInfo, MarkerInfoArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32
from tf.transformations import euler_from_quaternion 

class MarkerDistanceCalculator:
    def __init__(self):
        rospy.init_node('marker_distance_calculator')
        # subscribe to detected markers
        self.marker_sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.marker_callback)
        self.marker_info_pub = rospy.Publisher('/marker_info_array', MarkerInfoArray, queue_size=10)
        # TF Listener to get transforms between camera and base_link
        self.tf_listener = tf.TransformListener()

        self.br = tf.TransformBroadcaster()

    def marker_callback(self, data):
        markerinfo_array = MarkerInfoArray()
        # Loop through detected markers
        for marker in data.markers:

            # broadcast 
            marker_id = marker.id
            pos = marker.pose.position
            ori = marker.pose.orientation

            quaternion = (ori.x, ori.y, ori.z, ori.w)
            _, _, yaw = euler_from_quaternion(quaternion)  # yaw is the rotation around the z-axis

            marker_frame = "marker_{}".format(marker_id)
            self.br.sendTransform((pos.x, pos.y, pos.z),
                                  (ori.x, ori.y, ori.z, ori.w),
                                  rospy.Time.now(),
                                  marker_frame,
                                  marker.header.frame_id)

            # get marker pose in the camera frame 
            marker_pose = PoseStamped()
            marker_pose.header = marker.header
            marker_pose.pose = marker.pose

            try: 
                # transform marker pose from the camera frame to the robot base
                transformed_pose = self.tf_listener.transformPose("base_footprint", marker_pose)

                # Extract marker's position in robot's base frame
                marker_x = transformed_pose.pose.position.x
                marker_y = transformed_pose.pose.position.y

                # Get the 2D distance between marker and the robot base
                distance_2d = sqrt(marker_x**2 + marker_y**2)

                # Calculate the angle (theta) between the marker and robot's X-axis
                # This is the angle between the projected vector and the x direction of the robot
                theta = atan2(marker_y, marker_x)

                # Create a MarkerInfo message
                marker_info = MarkerInfo()
                marker_info.id = marker_id
                marker_info.distance = distance_2d
                marker_info.theta = theta
                marker_info.yaw = yaw
                marker_info.corners = marker.corners 
                markerinfo_array.markers.append(marker_info)

                rospy.loginfo("Marker ID: %d", marker.id)
                rospy.loginfo("2D Distance to robot base: %.2f meters", distance_2d)
                rospy.loginfo("Theta (angle to robot's front): %.2f radians", theta)
                rospy.loginfo("Yaw (Normal direction of marker): %.2f radians", yaw)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("TF transform failed")
        # Publish the marker info
        self.marker_info_pub.publish(markerinfo_array)


if __name__ == '__main__':
    try:
        MarkerDistanceCalculator()
        rospy.spin()
    except rospy.ROSInternalException:
        pass
