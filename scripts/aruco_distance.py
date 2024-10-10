import rospy
from math import sqrt, atan2
from enshu2.msg import MarkerArray, MarkerInfo, MarkerInfoArray

class MarkerDistanceCalculator:
    def __init__(self):
        rospy.init_node('marker_distance_calculator')

        # Subscribe to detected markers
        self.marker_sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.marker_callback)
        self.marker_info_pub = rospy.Publisher('/marker_info_array', MarkerInfoArray, queue_size=10)

    def marker_callback(self, data):
        markerinfo_array = MarkerInfoArray()

        # Loop through detected markers
        for marker in data.markers:
            marker_id = marker.id
            pos = marker.pose.position

            # Calculate the 2D distance directly from the marker position in the camera frame
            distance_2d = sqrt(pos.z**2 + pos.x**2)

            # Calculate the angle (theta) between the marker and camera's X-axis
            theta = atan2(pos.x, pos.z)

            # Create a MarkerInfo message
            marker_info = MarkerInfo()
            marker_info.id = marker_id
            marker_info.distance = distance_2d
            marker_info.theta = theta
            marker_info.corners = marker.corners  # Assuming you still need the corners data

            # Append the marker info to the array
            markerinfo_array.markers.append(marker_info)

            # Log information
            rospy.loginfo("Marker ID: %d", marker_id)
            rospy.loginfo("2D Distance to camera: %.2f meters", distance_2d)
            rospy.loginfo("Theta (angle to camera's X-axis): %.2f radians", theta)

        # Publish the marker info array
        self.marker_info_pub.publish(markerinfo_array)

if __name__ == '__main__':
    try:
        MarkerDistanceCalculator()
        rospy.spin()
    except rospy.ROSInternalException:
        pass
