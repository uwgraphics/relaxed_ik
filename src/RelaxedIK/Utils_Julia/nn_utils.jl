using Knet

function state_to_joint_pts_withreturn(x, vars)
    joint_pts = Array{Float64, 1}()
    # joint_pts = []
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
        x = Knet.relu.(w[i]*x + w[i+1])
    end
    return w[end-1]*x + w[end]
    # x = w[1]*x + w[2]
    # x = w[3]*x + w[4]
    #x = w[5]*x + w[6]
    #x = w[7]*x + w[8]
    #x = w[9]*x + w[10]
    #return x
end

function get_gradient_wrt_input(w, x)
    # returns gradient of the neural network, specified using the weights in w,
    #   with respect to the input x.  This only works for relu activation right now
    #   (the default option in the relaxedIK preprocessing).
    #  w: neural network layer weights, trained using relu activations
    #  x: input
    #  return: ∇
    ∇ = w[1]
    for i=1:2:length(w)
        v = w[i]*x + w[i+1]
        x = Knet.relu.(v)
        if ! (i == 1)
            ∇ = (ceil.(min.(max.(v,0.),1.))) .* w[i] * ∇
        else
            ∇ = (ceil.(min.(max.(v,0.),1.))) .* ∇
        end
    end
    return ∇, x
end

function get_rand_state_with_bounds(bounds)
    sample = []
    for b in bounds
        push!(sample, rand(Uniform(b[1], b[2])))
    end
    return sample
end


function get_rand_state_with_bounds(relaxedIK_vars)
    sample = Array{Float64,1}()
    bounds = relaxedIK_vars.vars.bounds
    for b in bounds
        push!(sample, rand(Uniform(b[1], b[2])))
    end
    return sample
end
