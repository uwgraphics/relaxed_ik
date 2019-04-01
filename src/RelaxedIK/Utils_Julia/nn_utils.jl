using Knet

function state_to_joint_pts_withreturn(x, vars)
    joint_pts = Array{Real, 1}()
    for i=1:vars.robot.num_chains
        vars.robot.arms[i].getFrames(x[vars.robot.subchain_indices[i]])
    end

    for j=1:vars.robot.num_chains
        out_pts = vars.robot.arms[j].out_pts
        for k = 1:length(out_pts)
            for l=1:3
                push!(joint_pts, out_pts[k][l])
            end
        end
    end
    return joint_pts
end

function state_to_joint_pts_inplace(x, vars)
    # joint_pts = []
    for i=1:vars.robot.num_chains
        vars.robot.arms[i].getFrames(x[vars.robot.subchain_indices[i]])
    end

    idx = 1
    for j=1:vars.robot.num_chains
        out_pts = vars.robot.arms[j].out_pts
        for k = 1:length(out_pts)
            for l=1:3
                # push!(joint_pts, out_pts[k][l])
                vars.joint_pts[idx] = out_pts[k][l]
                idx += 1
            end
        end
    end
end


function predict(w,x)
    for i=1:2:length(w)-2
        x = Knet.relu.(w[i]*x .+ w[i+1])
    end
    return w[end-1]*x .+ w[end]
end
