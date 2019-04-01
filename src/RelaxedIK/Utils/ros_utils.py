from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy


def draw_sphere_in_rviz(publisher, frame_id, position, scale, color, id=0):
    # marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 3)
    # publisher must publish a visualization_msg/Marker to "visualization_marker" topic
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()

    marker.type = 2
    marker.id = id

    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]

    publisher.publish(marker)


def draw_arrow_in_rviz(publisher, frame_id, start_point, end_point, shaft_diameter, head_diameter, color, id=1):
    # marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 3)
    # publisher must publish a visualization_msg/Marker to "visualization_marker" topic
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()

    marker.type = 0
    marker.id = id

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    marker.scale.x = shaft_diameter
    marker.scale.y = head_diameter
    marker.scale.z = 0.5

    p1 = Point()
    p2 = Point()

    p1.x = start_point[0]
    p1.y = start_point[1]
    p1.z = start_point[2]

    p2.x = end_point[0]
    p2.y = end_point[1]
    p2.z = end_point[2]

    marker.points.append(p1)
    marker.points.append(p2)

    publisher.publish(marker)


def draw_text_in_rviz(publisher, frame_id, text, position, scale_value, color, id=2):
    # marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 3)
    # publisher must publish a visualization_msg/Marker to "visualization_marker" topic
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()

    marker.type = 9
    marker.id = id

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    marker.scale.z = scale_value

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]

    marker.text = text

    publisher.publish(marker)



def draw_linestrip_in_rviz(publisher, frame_id, points, color, width = 0.03, id=3):
    # marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 3)
    # publisher must publish a visualization_msg/Marker to "visualization_marker" topic
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()

    marker.type = 4
    marker.id = id

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    marker.scale.x = width

    for i in xrange(len(points)):
        p = Point()
        p.x = points[i][0]
        p.y = points[i][1]
        p.z = points[i][2]
        marker.points.append(p)

    publisher.publish(marker)


def draw_linelist_in_rviz(publisher, frame_id, points, color, width = 0.03, id=3):
    # marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 3)
    # publisher must publish a visualization_msg/Marker to "visualization_marker" topic
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()

    marker.type = 5
    marker.id = id

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    marker.scale.x = width

    for i in xrange(len(points)):
        p = Point()
        p.x = points[i][0]
        p.y = points[i][1]
        p.z = points[i][2]
        marker.points.append(p)

    publisher.publish(marker)
