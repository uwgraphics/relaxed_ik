ENV["PYTHON"] = "/usr/bin/python"

println("preparing for preprocessing....")

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

function total_loss(ins, outs, m)
    total_error = 0.0
    for i=1:length(ins)
        total_error += Flux.mse(m(ins[i]), outs[i])
        # total_error += abs(Flux.Tracker.data(m(ins[i])) - outs[i])
    end
    return total_error
end

path_to_src = Base.source_dir()
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)

println("loaded robot: $loaded_robot")

relaxedIK = get_standard(path_to_src, loaded_robot; preconfigured=true)
cv = c.CollisionVars(path_to_src)

num_dof = relaxedIK.relaxedIK_vars.robot.num_dof

state_to_joint_pts_closure = (x) -> state_to_joint_pts(x, relaxedIK.relaxedIK_vars)

ins = []
outs = []

num_samples = 200
for i=1:num_samples
    # vel = 0.02 * rand(num_dof)
    #in = last_state + vel
    in = rand(Uniform(-6,6), num_dof)
    # last_state = in
    out = [c.get_score(in, cv)]
    println(in)

    push!(ins, state_to_joint_pts_closure(in))
    # push!(ins, in)
    push!(outs, out)
    println(out)

    println("sample $i of $num_samples")
end

data = Flux.zip(ins, outs)

nn_val = 70
m = Chain( Dense(length( state_to_joint_pts_closure( rand(num_dof) ) ), nn_val, Flux.relu),
    Dense(nn_val, nn_val, Flux.relu),
    Dense(nn_val, nn_val, Flux.relu),
    Dense(nn_val, nn_val, Flux.relu),
    Dense(nn_val, 1)
)

p = Flux.params(m)
opt = Flux.ADAM(p)
loss(x, y) = Flux.mse(m(x), y)
evalcb = () -> @show(total_loss(ins, outs, m))

@epochs 10000 Flux.train!( loss, data, opt, cb = Flux.throttle(evalcb, 4.0)  )

#=
nn_val = 70
m = Chain( Dense(length(ins[1]),nn_val, Flux.relu), Dense(nn_val,nn_val,Flux.relu), Dense(nn_val,1))

p = Flux.params(m)

opt = Flux.ADAM(p)
loss(x, y) = Flux.mse(m(x), y)
evalcb = () -> @show(total_loss(ins, outs, m))



@epochs 100 Flux.train!( loss, data, opt, cb = Flux.throttle(evalcb, 4.0)  )

println( m([0.6,1.0]) )
=#
