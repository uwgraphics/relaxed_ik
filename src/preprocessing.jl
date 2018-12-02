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
using CuArrays
import Distributions: Uniform
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/Utils_Julia/transformations.jl")

@pyimport RelaxedIK.Utils.collision_transfer as c

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

function total_loss(ins, outs, m)
    total_error = 0.0
    for i=1:length(ins)
        total_error += Flux.mse(m(ins[i]), outs[i])
    end
    return total_error
end

function run_preprocessing(num_samples=50000)

    path_to_src = Base.source_dir()
    loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
    loaded_robot = readline(loaded_robot_file)

    relaxedIK = get_standard(path_to_src, loaded_robot)
    cv = c.CollisionVars(path_to_src)

    num_dof = relaxedIK.relaxedIK_vars.robot.num_dof

    state_to_joint_pts_closure = (x) -> state_to_joint_pts(x, relaxedIK.relaxedIK_vars)

    ins = []
    outs = []

    test_ins = []
    test_outs = []

    # make samples
    # last_state = relaxedIK.relaxedIK_vars.vars.init_state
    for i=1:num_samples
        # vel = 0.02 * rand(num_dof)
        #in = last_state + vel
        in = rand(Uniform(-6,6), num_dof)
        # last_state = in
        out = [c.get_score(in, cv)]


        push!(ins, state_to_joint_pts_closure(in))
        # push!(ins, in)
        push!(outs, out)

        println("sample $i of $num_samples")
    end

    for i=1:100
        in = rand(Uniform(-6,6), num_dof)
        out = [c.get_score(in, cv)]

        push!(test_ins, state_to_joint_pts_closure(in))
        push!(test_outs, out)
    end

    data = zip(ins, outs)

    m = Chain(
        Dense(length(state_to_joint_pts_closure(rand(num_dof))), 70, Flux.relu),
        Dense(70, 70,Flux.relu),
        Dense(70, 70,Flux.relu),
        Dense(70, 70,Flux.relu),
        Dense(70, 70,Flux.relu),
        Dense(70, 1,Flux.relu)
    )

    p = Flux.params(m)
    opt = Flux.Optimise.ADAM(p, 0.001)

    loss(x, y) = Flux.mse(m(x), y)

    # evalcb = () -> @show(loss(ins[1], outs[1]))
    evalcb = () -> @show(total_loss(test_ins, test_outs, m))

    loss_before = total_loss(ins, outs, m)
    total_loss(ins, outs, m)
    epochs = 3
    for i = 1:epochs
        println("epoch $i of $epochs")
        Flux.train!( loss, data, opt, cb = Flux.throttle(evalcb, 6.0))
    end
    loss_after = total_loss(ins, outs, m)
    println("total loss before: $loss_before, total loss after: $loss_after")

    # save it
    f = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
    y = YAML.load(f)

    collision_nn_file_name = y["collision_nn_file"]

    @save path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name m
end

run_preprocessing()
