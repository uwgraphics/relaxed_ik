using Knet
using ForwardDiff
using Calculus
using LinearAlgebra

ENV["PYTHON"] = "/usr/bin/python"

using PyCall
using Flux
using Flux: @epochs
using YAML
using BSON: @save
using BSON
using Calculus
using ForwardDiff
using ReverseDiff
import Distributions: Uniform
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/Utils_Julia/transformations.jl")
@pyimport RelaxedIK.Utils.collision_transfer as c

function predict(w,x)
    for i=1:2:length(w)-2
        x = Knet.relu.(w[i]*x .+ w[i+1])
    end
    return w[end-1]*x .+ w[end]
end


function state_to_joint_pts(x, vars)
    joint_pts = []
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

function state_to_joint_pts_g(x, vars)
    joint_pts = []
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
    return norm(joint_pts[end])
end

function state_to_joint_pts2(x, vars, joint_pts)
    # joint_pts = []
    for i=1:vars.robot.num_chains
        vars.robot.arms[i].getFrames(x[vars.robot.subchain_indices[i]])
    end

    count = 1
    for j=1:vars.robot.num_chains
        out_pts = vars.robot.arms[j].out_pts
        for k = 1:length(out_pts)
            for l=1:3
                # push!(joint_pts, out_pts[k][l])
                joint_pts[count] = out_pts[k][l]
                count += 1
            end
        end
    end
    return joint_pts
end


path_to_src = Base.source_dir()
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)

println("loaded robot: $loaded_robot")

relaxedIK = get_standard(path_to_src, loaded_robot; preconfigured=true)

state_to_joint_pts_closure = (x) -> state_to_joint_pts(x, relaxedIK.relaxedIK_vars)

fp = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
y = YAML.load(fp)
collision_nn_file_name = y["collision_nn_file"]
w = BSON.load(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name)[:w]

model = (x) -> predict(w, x)[1]

# println( model(state_to_joint_pts_closure([0.,0.,0.,0.,0.,0.])  ) )

nn_func = (x) -> model(state_to_joint_pts_closure(x))

# println(nn_func([0.,0.,0.,0.,0.,0.]))

g = (x) -> Calculus.gradient(nn_func, x)

relaxedIK = get_autocam1(path_to_src, loaded_robot)
# gr = relaxedIK.relaxedIK_vars.vars.∇s[end]
# println(gr(rand(14)))
# @btime gr(rand(14))
# @btime model( state_to_joint_pts_closure(rand(14) )  )

#=
for i = 1:100
    println(solve(relaxedIK, [[1.,0.,0.]], [Quat(1.,0.,0.,0.)]))
end
=#

println( Array{Number, 1}() )

println(relaxedIK.relaxedIK_vars.vars.∇s[end](rand(14)))
@btime relaxedIK.relaxedIK_vars.vars.∇s[end](rand(14))
@btime relaxedIK.relaxedIK_vars.vars.objective_closures[end](rand(14))

# @btime relaxedIK.relaxedIK_vars.vars.objective_closures[end](rand(14))
# @btime g( rand(14) )

# @btime relaxedIK.relaxedIK_vars.vars.objective_closures[end]([0.,0.,0.,0.,0.,0.])
# jt_pts = state_to_joint_pts([0.,0.,0.,0.,0.,0.], relaxedIK.relaxedIK_vars)
# @btime predict(w, rand(54))
# @btime relaxedIK.relaxedIK_vars.nn_model( a )
# @btime state_to_joint_pts( rand(14) , relaxedIK.relaxedIK_vars)
# jp = relaxedIK.relaxedIK_vars.joint_pts
# @btime relaxedIK.relaxedIK_vars.nn_model( jp )
@btime relaxedIK.relaxedIK_vars.nn_model( relaxedIK.relaxedIK_vars.joint_pts )
# @btime relaxedIK.relaxedIK_vars.vars.objective_closures[end](rand(14))

# println( state_to_joint_pts_g(rand(14) , relaxedIK.relaxedIK_vars) )
# gt1 = (x) -> state_to_joint_pts_g(x, relaxedIK.relaxedIK_vars)
# gt = (x) -> ForwardDiff.gradient(gt1, x)
# println(gt(rand(14)))
# @btime state_to_joint_pts( rand(14) , relaxedIK.relaxedIK_vars)
# @btime state_to_joint_pts2([0.,0.,0.,0.,0.,0.], relaxedIK.relaxedIK_vars, jt_pts)
# @btime gt(rand(14))
