from geometry_msgs.msg import PolygonStamped, Point32, Pose, Quaternion
from visualization_msgs.msg import Marker
import rospy

def create_shape(ns, id, pt, size=0.1, rot=[0, 0, 0, 1], color=[1, 0, 0]):
    marker = Marker()
    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time.now()

    marker.ns = ns
    marker.id = id
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = 0.001
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    s = Point32()
    s.x = pt[0]
    s.y = pt[1]
    s.z = pt[2]

    r = Quaternion()
    r.x = rot[0]
    r.y = rot[1]
    r.z = rot[2]
    r.w = rot[3]

    p = Pose()
    p.position = s
    p.orientation = r

    marker.pose = p

    return marker


def delete_shape(ns, id):
    marker = Marker()
    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = marker.ARROW
    marker.action = marker.DELETE

    return marker

def create_axis(ns, id, pos, ori):
    arrow_x = create_arrow("axis_x_"+ns, id, pos, pos+ori.distance(0.1, 0, 0), color=[1, 0, 0], size=[0.01, 0.02, 0.02])
    arrow_y = create_arrow("axis_y_"+ns, id, pos, pos+ori.distance(0, 0.1, 0), color=[0, 1, 0], size=[0.01, 0.02, 0.02])
    arrow_z = create_arrow("axis_z_"+ns, id, pos, pos+ori.distance(0, 0, 0.1), color=[0, 0, 1], size=[0.01, 0.02, 0.02])

    return [arrow_x, arrow_y, arrow_z]

def create_arrow(ns, id, start, end, color=[1, 0, 0], size=[0.02, 0.05, 0.05]):
    marker = Marker()
    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = size[0]
    marker.scale.y = size[1]
    marker.scale.z = size[2]
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


def create_arrow2(ns, id, start, rot, dist, color=[1, 0, 0]):
    marker = Marker()
    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = 0.02
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    end = start+rot.distance(dist[0], dist[1], dist[2])

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

def create_polygon_list(points):
    p = PolygonStamped()
    p.header.frame_id = "base"
    p.header.stamp = rospy.Time.now()

    for b in points:
        point = Point32()
        point.x = b[0]
        point.y = b[1]
        point.z = b[2]
        p.polygon.points.append( point )

    return p