using Rotations

pos_goals = []
quat_goals = []
function eePoseGoals_cb(data::EEPoseGoals)
    global pos_goals, quat_goals
    eepg = data

    pose_goals = eepg.ee_poses

    pos_goals = []
    quat_goals = []

    for i = 1:length(pose_goals)
        p = pose_goals[i]

        pos_x = p.position.x
        pos_y = p.position.y
        pos_z = p.position.z

        quat_w = p.orientation.w
        quat_x = p.orientation.x
        quat_y = p.orientation.y
        quat_z = p.orientation.z

        push!(pos_goals, [pos_x, pos_y, pos_z])
        push!(quat_goals, Quat(quat_w, quat_x, quat_y, quat_z))
    end
end
# Subscriber{EEPoseGoals}("/relaxed_ik/ee_pose_goals", eePoseGoals_cb)


# timing for this was about 0.01 - 0.02 for about 3000 points
point_cloud2 = ""
function point_cloud2_cb(data)
    global point_cloud2
    point_cloud2 = []
    x = pc2[:read_points](data)
    for i in x
        push!(point_cloud2, i)
    end
end
# Subscriber{PointCloud2}("collision_point_cloud", point_cloud2_cb)


# (Data ready to go for kdtree implementation from Neighbors.jl)
# timing for this was about 0.01 - 0.02 for about 3000 points
point_cloud2 = ""
function point_cloud2_cb(data)
    global point_cloud2
    x = pc2[:read_points](data)
    count = 0
    for i in enumerate(x)
        count += 1
    end
    point_cloud2 = zeros(Float64,3,count)
    for (index, i) in enumerate(x)
        point_cloud2[1,index] = i[1]
        point_cloud2[2,index] = i[2]
        point_cloud2[3,index] = i[3]
    end
    #kdtree = KDTree(point_cloud2)
    # println(knn(kdtree, [0.0, 0.0, 0.0], 1, true))
end
# Subscriber{PointCloud2}("collision_point_cloud", point_cloud2_cb)



b = false
function bool_cb(data::BoolMsg)
    global b
    b = data.data
end
# Subscriber{BoolMsg}("b", bool_cb)
