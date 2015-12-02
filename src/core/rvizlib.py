from geometry_msgs.msg import PolygonStamped, Point32, Pose, Quaternion
from visualization_msgs.msg import Marker, InteractiveMarkerControl, InteractiveMarker
import rospy, genpy

def create_shape(holder, ns, id, pt, size=0.1, rot=[0, 0, 0, 1], color=[1, 0, 0]):
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
    marker.lifetime = genpy.Duration(0.1)

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

    holder.markers.append(marker)

def delete_shape(holder, ns, id):
    marker = Marker()
    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = marker.ARROW
    marker.action = marker.DELETE

def create_axis(holder, ns, id, pos, ori):
    create_arrow(holder, "axis_x_"+ns, id, pos, pos+ori.distance(0.1, 0, 0), color=[1, 0, 0], size=[0.01, 0.02, 0.02])
    create_arrow(holder, "axis_y_"+ns, id, pos, pos+ori.distance(0, 0.1, 0), color=[0, 1, 0], size=[0.01, 0.02, 0.02])
    create_arrow(holder, "axis_z_"+ns, id, pos, pos+ori.distance(0, 0, 0.1), color=[0, 0, 1], size=[0.01, 0.02, 0.02])

def create_arrow(holder, ns, id, start, end, color=[1, 0, 0], size=[0.02, 0.05, 0.05]):
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
    marker.lifetime = genpy.Duration(0.1)

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

    holder.markers.append(marker)


def create_arrow2(holder, ns, id, start, rot, dist, color=[1, 0, 0]):
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
    marker.lifetime = genpy.Duration(0.1)

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

    holder.markers.append(marker)

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

def create_interactive_marker(holder, id, pos, size, color, func):
    # Make interactive marker for mouse selection
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base"
    int_marker.name = "object"+str(id)
    int_marker.pose.position.x = pos[0]
    int_marker.pose.position.y = pos[1]
    int_marker.pose.position.z = pos[2]

    #color = [1, 0, 0]

    # Add click control
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.interaction_mode = InteractiveMarkerControl.BUTTON

    create_shape(box_control, "object", id, pos, size=size, color=color)

    # add the control to the interactive marker
    int_marker.controls.append( box_control )
    holder.insert(int_marker, func)
