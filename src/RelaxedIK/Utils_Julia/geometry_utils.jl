using LinearAlgebra
using Distances

function closest_point_on_2_lines(u1, u2, v1, v2)
    u = u2 - u1
    v = v2 - v1
    rho = v1 - u1
    uv = dot(u,v)
    uu = dot(u,u)
    vv = dot(v, v)
    urho = dot(u, rho)
    vrho = dot(v, rho)

    vt = (vrho*uu - urho*uv) / (uv*uv - vv*uu)
    ut = (uv * vt + urho) / uu

    if isnan(vt) || isinf(vt)
        vt = 0.0
    end

    if isnan(ut) || isinf(ut)
        ut = 0.0
    end

    ut = min( max(0.,ut), 1.)
    vt = min( max(0.,vt), 1.)

    return ut, vt, u1 + ut*(u2 - u1), v1 + vt*(v2 - v1)
end

function dis_between_line_segments(u1, u2, v1, v2)
    ut,vt,p1,p2 = closest_point_on_2_lines( u1,u2,v1,v2 )
    return norm(p1-p2)
end

function pt_dis_to_line_seg(pt::Array{Float64, 1}, a::Array{Float64, 1}, b::Array{Float64, 1})
    u = dot((pt - a), (b - a)) / max(norm(a-b)^2, 0.00000000000001)
    u = min( max(u,0.) , 1.)
    p = a + u*(b - a)
    dis = norm(p - pt)
    return dis, p
end

function pt_dis_to_line(pt::AbstractArray, a::AbstractArray, b::AbstractArray)
    u = dot((pt - a), (b - a)) / euclidean(a,b)^2
    # u = min( max(u,0.) , 1.)
    p = a + u*(b - a)
    # dis = norm(p - pt)
    dis = euclidean(p, pt)
    return dis, p
end

function pt_dis_to_line_and_segment(pt::AbstractArray, a::AbstractArray, b::AbstractArray)
    u1 = dot((pt - a), (b - a)) / euclidean(a,b)^2
    u2 = min( max(u1,0.) , 1.)
    p1 = a + u1*(b - a)
    p2 = a + u2*(b - a)

    # dis = norm(p - pt)
    dis1 = euclidean(p1, pt)
    dis2 = euclidean(p2, pt)
    return dis1, p1, u1, dis2, p2, u2
end
