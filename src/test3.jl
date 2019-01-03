using Knet
using ForwardDiff
using Calculus

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
    # return x
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

g = (x) -> ForwardDiff.gradient(nn_func, x)


relaxedIK = get_standard(path_to_src, loaded_robot)
# println(relaxedIK.relaxedIK_vars.vars.∇s[end]([0.,0.,0.,0.,0.,0.]))

for i = 1:100
    println(solve(relaxedIK, [[1.,0.,0.]], [Quat(1.,0.,0.,0.)]))
end
