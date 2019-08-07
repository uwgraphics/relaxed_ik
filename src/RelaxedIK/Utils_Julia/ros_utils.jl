
function draw_sphere_in_rviz(publisher, frame_id, position, scale, color; id=0)
    # marker_pub = Publisher("/visualization_marker", Marker, queue_size = 3)
    # using RobotOS
    # @rosimport visualization_msgs.msg: Marker
    # rostypegen()
    # using .visualization_msgs.msg
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = RobotOS.now()

    marker.type = 2
    marker.id = id


    marker.scale.x = scale[1]
    marker.scale.y = scale[2]
    marker.scale.z = scale[3]

    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    marker.color.a = color[4]

    marker.pose.position.x = position[1]
    marker.pose.position.y = position[2]
    if length(position) > 2
        marker.pose.position.z = position[3]
    else
        marker.pose.position.z = 0.0
    end

    publish(publisher, marker)
end

function draw_cube_in_rviz(publisher, frame_id, position, scale, color; id=0)
    # marker_pub = Publisher("/visualization_marker", Marker, queue_size = 3)
    # using RobotOS
    # @rosimport visualization_msgs.msg: Marker
    # rostypegen()
    # using .visualization_msgs.msg
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = RobotOS.now()

    marker.type = 1
    marker.id = id


    marker.scale.x = scale[1]
    marker.scale.y = scale[2]
    marker.scale.z = scale[3]

    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    marker.color.a = color[4]

    marker.pose.position.x = position[1]
    marker.pose.position.y = position[2]
    if length(position) > 2
        marker.pose.position.z = position[3]
    else
        marker.pose.position.z = 0.0
    end

    publish(publisher, marker)
end

function draw_arrow_in_rviz(publisher, frame_id, start_point, end_point, shaft_diameter, head_diameter, color; id=1)
    # marker_pub = Publisher("/visualization_marker", Marker, queue_size = 3)
    # @rosimport visualization_msgs.msg: Marker
    # @rosimport geometry_msgs.msg: Point
    # rostypegen()
    # using .visualization_msgs.msg
    # using .geometry_msgs.msg
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = RobotOS.now()

    marker.type = 0
    marker.id = id

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
    if length(start_point) > 2
        p1.z = start_point[3]
    else
        p1.z = 0.0
    end

    p2.x = end_point[1]
    p2.y = end_point[2]
    if length(end_point) > 2
        p2.z = end_point[3]
    else
        p2.z = 0.0
    end

    push!(marker.points, p1)
    push!(marker.points, p2)

    publish(publisher, marker)
end

function draw_text_in_rviz(publisher, frame_id, text, position, scale_value, color; id=2)
    # marker_pub = Publisher("/visualization_marker", Marker, queue_size = 3)
    # @rosimport visualization_msgs.msg: Marker
    # rostypegen()
    # using .visualization_msgs.msg
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = RobotOS.now()

    marker.type = 9
    marker.id = id


    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    marker.color.a = color[4]

    marker.scale.z = scale_value

    marker.pose.position.x = position[1]
    marker.pose.position.y = position[2]
    if length(position) > 2
        marker.pose.position.z = position[3]
    else
        marker.pose.position.z = 0.0
    end

    marker.text = text

    publish(publisher, marker)
end

function draw_linestrip_in_rviz(publisher, frame_id, points, color; id=3, width = 0.03)
    # marker_pub = Publisher("/visualization_marker", Marker, queue_size = 3)
    # @rosimport visualization_msgs.msg: Marker
    # @rosimport geometry_msgs.msg: Point
    # rostypegen()
    # using .visualization_msgs.msg
    # using .geometry_msgs.msg
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = RobotOS.now()

    marker.type = 4
    marker.id = id


    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    marker.color.a = color[4]

    marker.scale.x = width

    for i = 1:length(points)
        p = Point()
        p.x = points[i][1]
        p.y = points[i][2]
        if length(points[i]) > 2
            p.z = points[i][3]
        else
            p.z = 0.0
        end
        push!(marker.points, p)
    end

    publish(publisher, marker)
end

function draw_linelist_in_rviz(publisher, frame_id, points, color; id=3, width = 0.03)
    # marker_pub = Publisher("/visualization_marker", Marker, queue_size = 3)
    # @rosimport visualization_msgs.msg: Marker
    # @rosimport geometry_msgs.msg: Point
    # rostypegen()
    # using .visualization_msgs.msg
    # using .geometry_msgs.msg
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = RobotOS.now()

    marker.type = 5
    marker.id = id

    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    marker.color.a = color[4]

    marker.scale.x = width

    for i = 1:length(points)
        p = Point()
        p.x = points[i][1]
        p.y = points[i][2]
        if length(points[i]) > 2
            p.z = points[i][3]
        else
            p.z = 0.0
        end
        push!(marker.points, p)
    end

    publish(publisher, marker)
end

function draw_cube_list_in_rviz(publisher, frame_id, points, scale, color; id=4)
    # marker_pub = Publisher("/visualization_marker", Marker, queue_size = 3)
    # @rosimport visualization_msgs.msg: Marker
    # @rosimport geometry_msgs.msg: Point
    # rostypegen()
    # using .visualization_msgs.msg
    # using .geometry_msgs.msg
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = RobotOS.now()

    marker.type = 6
    marker.id = id

    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    marker.color.a = color[4]

    marker.scale.x = scale[1]
    marker.scale.y = scale[2]
    marker.scale.z = scale[3]

    for i = 1:length(points)
        p = Point()
        p.x = points[i][1]
        p.y = points[i][2]
        if length(points[i]) > 2
            p.z = points[i][3]
        else
            p.z = 0.0
        end
        push!(marker.points, p)
    end

    publish(publisher, marker)
end
