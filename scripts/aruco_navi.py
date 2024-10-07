import rospy
import tf
from math import sqrt, atan2
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from enshu4.msg import MarkerInfo
from geometry_msgs.msg import Point32

class MarkerDistanceCalculator:
    def __init__(self):
        rospy.init_node('marker_distance_calculator')
        # subscribe to detected markers
        self.marker_sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.marker_callback)
        self.marker_info_pub = rospy.Publisher('/marker_info', MarkerInfo, queue_size=10)
        # TF Listener to get transforms between camera and base_link
        self.tf_listener = tf.TransformListener()

        self.br = tf.TransformBroadcaster()

    def marker_callback(self, data):
        # Loop through detected markers
        for marker in data.markers:

            # broadcast 
            marker_id = marker.id
            pos = marker.pose.pose.position
            ori = marker.pose.pose.orientation

            marker_frame = "marker_{}".format(marker_id)
            self.br.sendTransform((pos.x, pos.y, pos.z),
                                  (ori.x, ori.y, ori.z, ori.w),
                                  rospy.Time.now(),
                                  marker_frame,
                                  marker.header.frame_id)

            # get marker pose in the camera frame 
            marker_pose = PoseStamped()
            marker_pose.header = marker.header
            marker_pose.pose = marker.pose.pose

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
                # Assuming the corners of the marker are available in marker.corners[0]
                corners = marker.corners[0]  # This will give you the 4 corners of the marker
            
                # Save each corner in the MarkerInfo message
                for corner in corners:
                    corner_point = Point32()
                    corner_point.x = corner[0]  # x coordinate
                    corner_point.y = corner[1]  # y coordinate
                    corner_point.z = 0  # You can set z to 0 for 2D image coordinates
                    marker_info.corners.append(corner_point)

                # Publish the marker info
                self.marker_info_pub.publish(marker_info)

                rospy.loginfo("Marker ID: %d", marker.id)
                rospy.loginfo("2D Distance to robot base: %.2f meters", distance_2d)
                rospy.loginfo("Theta (angle to robot's front): %.2f radians", theta)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("TF transform failed")

if __name__ == '__main__':
    try:
        MarkerDistanceCalculator()
        rospy.spin()
    except rospy.ROSInternalException:
        pass
