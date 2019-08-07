using NearestNeighbors
using LinearAlgebra
include("geometry_utils.jl")
include("../relaxedIK.jl")

function get_point_cloud_from_files(path_to_src, file_names)
    pts = []
    for p in file_names
        fp = path_to_src  * "/RelaxedIK/FileIO/point_clouds/$p"
        f = open(fp)

        line = readline(f)
        while true
            if line == ""
                break
            end
            line_arr = split(line, ",")
            push!(pts, [ parse(Float64, line_arr[1]), parse(Float64, line_arr[2]), parse(Float64, line_arr[3]) ] )

            line = readline(f)
        end
    end
    return list_of_points_to_matrix_of_points(pts)
end

function list_of_points_to_matrix_of_points(list_of_points)
    num_pts = length(list_of_points)
    mat = zeros(3, num_pts)
    for i = 1:num_pts
        mat[1,i] = list_of_points[i][1]
        mat[2,i] = list_of_points[i][2]
        mat[3,i] = list_of_points[i][3]
    end
    return mat
end

function list_of_points_to_matrix_of_points_2d(list_of_points)
    num_pts = length(list_of_points)
    mat = zeros(2, num_pts)
    for i = 1:num_pts
        mat[1,i] = list_of_points[i][1]
        mat[2,i] = list_of_points[i][2]
    end
    return mat
end

function list_of_points_to_matrix_of_points_nd(list_of_points)
    num_pts = length(list_of_points)
    mat = zeros(length(list_of_points[1]), num_pts)
    for i = 1:num_pts
        for j = 1:length(list_of_points[i])
            mat[j,i] = list_of_points[i][j]
        end
    end
    return mat
end

function get_kdtree_from_point_cloud(point_cloud_data)
    # point cloud data is assumed to already be in matrix format from the second cb
    return KDTree(point_cloud_data)
end

function get_nearest_neighbors(kdtree, point, point_cloud_data; k=1)
    idxs, dists = knn(kdtree, point, k, true)
    nn = []
    for i = 1:k
        push!(nn, point_cloud_data[:, idxs[i]])
    end
    return nn, dists
end

function get_nearest_neighbors_with_idxs(kdtree, point, point_cloud_data; k=1)
    idxs, dists = knn(kdtree, point, k, true)
    nn = []
    for i = 1:k
        push!(nn, point_cloud_data[:, idxs[i]])
    end
    return nn, dists, idxs
end

function get_robot_distance_from_point_cloud(x, relaxedIK, kdtree, point_cloud_data)
    joint_pts = get_joint_positions(relaxedIK, x)

    min_dis = 10000000000000.0
    closest_robot_pt = [0.,0.,0.]
    closest_cloud_pt = [0.,0.,0.]

    for k = 1:length(joint_pts)
        for i = 1:length(joint_pts[k]) - 1
            dis = norm(joint_pts[k][i] - joint_pts[k][i+1])
            num_pts = convert(Int32, max( ceil(dis*2.5^2), 2.0) )
            l = range( joint_pts[k][i], joint_pts[k][i+1], length = num_pts  )
            for j = 1:num_pts
                nn, dists = get_nearest_neighbors(kdtree, l[j], point_cloud_data)
                if dists[1] < min_dis
                    min_dis = dists[1]
                    closest_robot_pt = l[j]
                    closest_cloud_pt = nn[1]
                end
            end
        end
    end
    return closest_cloud_pt, closest_robot_pt, min_dis
end

function is_robot_colliding_with_point_cloud(x, relaxedIK, kdtree, point_cloud_data; collision_radius = 0.05)
    closest_cloud_pt, closest_robot_pt, min_dis = get_robot_distance_from_point_cloud(x, relaxedIK, kdtree, point_cloud_data)
    if min_dis < collision_radius
        return true
    else
        return false
    end
end

function is_point_colliding_with_point_cloud(x, kdtree, point_cloud_data; collision_radius = 0.05)
    nn, dists = get_nearest_neighbors(kdtree, x, point_cloud_data, k=1)
    if dists[1] < collision_radius
        return true
    else
        return false
    end
end
