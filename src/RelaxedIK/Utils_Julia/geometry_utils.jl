using LinearAlgebra

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

function pt_dis_to_line_seg(pt, a, b)
    u = dot((pt - a), (b - a)) / max(norm(a-b)^2, 0.00000000000001)
    u = min( max(u,0.) , 1.)
    p = a + u*(b - a)
    dis = norm(p - pt)
    return dis
end
