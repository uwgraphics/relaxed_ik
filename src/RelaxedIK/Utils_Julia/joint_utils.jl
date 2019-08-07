

function interpolate_to_joint_limits(from_q, to_q; t=0.1, joint_velocity_limits=nothing)
    #=
    interpolates from state from_q to sate to_q going as fast as the joints can in time t
    :param from_q:
    :param to_q:
    :param time:
    :param joint_velocity_limits:
    :return:
    =#
    ret_q = Array{Float64,1}()
    ret_k = Array{Float64,1}()
    numDOF = length(from_q)

    if joint_velocity_limits == nothing
        limits = [ ]
        for i = 1:numDOF
            push!(limits, 6.0)
        end
    else
        limits = joint_velocity_limits
    end

    for i=1:numDOF
        disp = to_q[i] - from_q[i]
        disp_norm = abs(disp)

        # how far can joint go in the amount of time?
        max_disp = limits[i]*t

        if disp_norm > max_disp
            k = max_disp/disp_norm
        else
            k = 1.0
        end

        push!(ret_k, k)
        # ret_k.append(k)
        interp = from_q[i] + k*(disp)
        push!(ret_q, interp)
        # ret_q.append(interp)
    end

    return ret_q, ret_k
end


function check_legal_velocity(from_q, to_q; t=0.1, joint_velocity_limits=nothing)
    q,k = interpolate_to_joint_limits(from_q,to_q, t = t, joint_velocity_limits=joint_velocity_limits)
    numDOF = length(q)
    if ! (k == ones(numDOF))
        return false
    else
        return true
    end
end
