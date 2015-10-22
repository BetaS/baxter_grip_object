from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
import rospy

def create_arrow(id, start, end, color=[1, 0, 0]):
    marker = Marker()
    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time.now()
    marker.id = id
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    s = Point32()
    s.x = start[0]
    s.y = start[1]
    s.z = start[2]
    e = Point32()
    e.x = end[0]
    e.y = end[1]
    e.z = end[2]
    marker.points.append(s)
    marker.points.append(e)

    return marker