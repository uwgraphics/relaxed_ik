#!/usr/bin/env julia

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

function get_rand_state_with_bounds(bounds)
    sample = []
    for b in bounds
        push!(sample, rand(Uniform(b[1], b[2])))
    end
    return sample
end

function run_preprocessing(num_samples=10000)

    path_to_src = Base.source_dir()
    loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
    loaded_robot = readline(loaded_robot_file)

    println("loaded robot: $loaded_robot")

    relaxedIK = get_standard(path_to_src, loaded_robot; preconfigured=true)
    cv = c.CollisionVars(path_to_src)

    num_dof = relaxedIK.relaxedIK_vars.robot.num_dof

    # println(num_dof)

    state_to_joint_pts_closure = (x) -> state_to_joint_pts(x, relaxedIK.relaxedIK_vars)

    ins = []
    outs = []

    test_ins = []
    test_outs = []
    data = []
    test_data = []

    # make samples
    # last_state = relaxedIK.relaxedIK_vars.vars.init_state
    for i=1:num_samples
        # vel = 0.02 * rand(num_dof)
        #in = last_state + vel
        # in = rand(Uniform(-6,6), num_dof)
        in = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
        # last_state = in
        out = [c.get_score(in, cv)]

        push!(ins, state_to_joint_pts_closure(in))
        # push!(ins, in)
        push!(outs, out)
        push!(data, (state_to_joint_pts_closure(in), out) )
        # println(out)

        println("sample $i of $num_samples")
    end


    for i=1:100
        # in = rand(Uniform(-6,6), num_dof)
        in = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
        out = c.get_score(in, cv)

        push!(test_ins, state_to_joint_pts_closure(in))
        push!(test_outs, out)
        push!(test_data, (state_to_joint_pts_closure(in), out) )
    end

    data = Flux.zip(ins, outs)

    nn_val = 40
    m = Chain( Dense(length( state_to_joint_pts_closure( rand(num_dof) ) ), nn_val, Flux.leakyrelu),
        Dropout(0.6),
        Dense(nn_val, nn_val, Flux.leakyrelu),
        Dropout(0.6),
        Dense(nn_val, nn_val, Flux.leakyrelu),
        Dropout(0.6),
        Dense(nn_val, nn_val, Flux.leakyrelu),
        Dropout(0.6),
        Dense(nn_val, nn_val, Flux.leakyrelu),
        Dense(nn_val, nn_val, Flux.leakyrelu),
        Dense(nn_val, nn_val, Flux.leakyrelu),
        Dense(nn_val, 1)
    )

    p = Flux.params(m)
    opt = Flux.Optimise.ADAM(p)

    loss(x, y) = Flux.mse(m(x), y)
    # loss(x,y) = m(x) - y

    # evalcb = () -> @show(loss(ins[1], outs[1]))
    # evalcb = () -> @show(total_loss(test_ins, test_outs, m))

    function evalcb(ins, outs, test_ins, test_outs, m)
        train_loss = total_loss(ins, outs, m)
        test_loss = total_loss(test_ins, test_outs, m)
        println("train set loss: $train_loss, test set loss: $test_loss")
    end

    evalcb_closure = () -> evalcb(ins[1:100], outs[1:100], test_ins, test_outs, m)

    loss_before = total_loss(ins, outs, m)
    # epochs = 11
    # for i = 1:epochs
    # println("epoch $i of $epochs")
    @epochs 30 Flux.train!( loss, data, opt, cb = Flux.throttle(evalcb_closure, 2.0))
    # end
    loss_after = total_loss(ins, outs, m)
    println("total loss before: $loss_before, total loss after: $loss_after")

    for i=1:length(test_ins)
        input = test_ins[i]
        # println(input)
        modeled_out =   m( input )
        # modeled_out = Flux.Tracker.data( m(state_to_joint_pts_closure(test_ins[i])) )[1]
        y_out = test_outs[i]

        random_output = m(  rand( length(  state_to_joint_pts_closure(test_ins[i]) ) ) )
        println("modeled_out: $modeled_out, y_out: $y_out")
    end

    # save it
    f = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
    y = YAML.load(f)

    collision_nn_file_name = y["collision_nn_file"]

    # @save path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name m
end

run_preprocessing()
