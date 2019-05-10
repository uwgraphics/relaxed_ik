using Rotations, StaticArrays, LinearAlgebra

q = Quat(1.0,0.0,0.0,0.0)
q2 = Quat(0.0,1.0,0.0,0.0)

function quaternion_log(quaternion)
    out_vec = MVector(quaternion.x, quaternion.y, quaternion.z)
    if abs(quaternion.w) < 1.0
        a = 1
        a = acos(quaternion.w)
        sina = sin(a)
        if abs(sina >= 0.005)
            c = a/sina
            out_vec[1] *= c
            out_vec[2] *= c
            out_vec[3] *= c
        end
    end
    return out_vec
end

function quaternion_exp(vec3)
    q = MVector(1.0, vec3[1], vec3[2], vec3[3])
    a = norm(q)
    sina = sin(a)
    if abs(sina) >= 0.005
        c = sina/a
        q[2]*=c
        q[3]*=c
        q[4]*=c
    end
    q[1] = cos(a)
    return Quat(q[1], q[2], q[3], q[4])
end

function quaternion_disp(q, qPrime)
    return quaternion_log(inv(q)*qPrime)
end

function quaternion_dispQ(q, qPrime)
    return inv(q)*qPrime
end

# using BenchmarkTools

# @btime quaternion_dispQ(q, q2)
