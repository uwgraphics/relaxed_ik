
function draw_sphere_in_rviz(publisher, frame_id, position, scale, color; id=0)
    # publisher must publisher a visualization_msg/Marker to "visualization_marker" topic
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = RobotOS.now()

    marker.type = 2
    marker.id = id
    marker.header.stamp = RobotOS.now()
    marker.header.frame_id = frame_id

    marker.scale.x = scale[1]
    marker.scale.y = scale[2]
    marker.scale.z = scale[3]

    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    marker.color.a = color[4]

    marker.pose.position.x = position[1]
    marker.pose.position.y = position[2]
    marker.pose.position.z = position[3]

    publish(publisher, marker)
end


function draw_arrow_in_rviz(publisher, frame_id, start_point, end_point, shaft_diameter, head_diameter, color; id=1)
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = RobotOS.now()

    marker.type = 0
    marker.id = id
    marker.header.stamp = RobotOS.now()
    marker.header.frame_id = frame_id

    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    marker.color.a = color[4]

    marker.scale.x = shaft_diameter
    marker.scale.y = head_diameter
    marker.scale.z = 0.5

    p1 = Point()
    p2 = Point()

    p1.x = start_point[1]
    p1.y = start_point[2]
    p1.z = start_point[3]

    p2.x = end_point[1]
    p2.y = end_point[2]
    p2.z = end_point[3]

    push!(marker.points, p1)
    push!(marker.points, p2)

    publish(publisher, marker)
end
