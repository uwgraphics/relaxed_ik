using BSON, YAML, Flux
using BenchmarkTools
using ForwardDiff
using Calculus
include("RelaxedIK/relaxedIK.jl")


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

path_to_src = Base.source_dir()
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)
f = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
y = YAML.load(f)
collision_nn_file_name = y["collision_nn_file"]
relaxedIK = get_standard(path_to_src, loaded_robot)

state_to_joint_pts_closure = (x) -> state_to_joint_pts(x, relaxedIK.relaxedIK_vars)


model = BSON.load(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name)[:m]

xr = [0.,0.,2.52,0.,0.,0.]

function f1(x, model)
    return Flux.Tracker.data(model(state_to_joint_pts_closure(x))[1])
end

f = (x)-> f1(x, model)
g = (x) -> ForwardDiff.gradient(f, x)

println(g(xr))
