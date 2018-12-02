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
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/Utils_Julia/transformations.jl")

@pyimport RelaxedIK.Utils.collision_transfer as c

function run_preprocessing(num_samples=200000)

    function state_to_joint_pts(x, vars)
        joint_pts = []
        for i=1:vars.robot.num_chains
            vars.robot.arms[i].getFrames(x[relaxedIK.relaxedIK_vars.robot.subchain_indices[i]])
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
        println("total error: $total_error")
    end

    path_to_src = Base.source_dir()
    loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
    loaded_robot = readline(loaded_robot_file)

    relaxedIK = get_standard(path_to_src, loaded_robot)
    cv = c.CollisionVars(path_to_src)

    num_dof = relaxedIK.relaxedIK_vars.robot.num_dof

    state_to_joint_pts_closure = (x) -> state_to_joint_pts(x, relaxedIK.relaxedIK_vars)

    ins = []
    outs = []


    # make samples
    last_state = relaxedIK.relaxedIK_vars.vars.init_state
    for i=1:num_samples
        # vel = 0.02 * rand(num_dof)
        # in = last_state + vel
        in = rand(num_dof)
        last_state = in
        out = c.get_score(in, cv)

        push!(ins, state_to_joint_pts_closure(in))
        push!(outs, out)

        println("sample $i of $num_samples")
    end

    data = zip(ins, outs)

    m = Chain(
        Dense(length(state_to_joint_pts_closure(rand(num_dof))), 70),
        Dense(70, 70),
        Dense(70, 70),
        Dense(70, 70),
        Dense(70, 70),
        Dense(70, 1)
    )

    p = Flux.params(m)
    opt = Flux.Optimise.ADAM(p)

    loss(x, y) = Flux.mse(m(x), y)

    #function loss_original(x,y)
    #    return ((Flux.Tracker.data(x)[1]) - y)^2
    #end

    # loss(x,y) = loss_original(m(x), y)

    # evalcb() = @show(loss(ins[1], outs[1]))

    # println(loss(rand(6), 0.00001))


    # @epochs 10 Flux.train!(loss, data, opt, cb = () ->  Flux.throttle( total_loss(ins, outs, m), 5) )

    function run_training()
        Flux.train!( loss, data, opt )
        total_loss(ins, outs, m)
    end

    @epochs 10 run_training()

    # save it
    f = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
    y = YAML.load(f)

    for i =1:10
        test = rand(num_dof)
        println(c.get_score(test, cv))
        println(m(state_to_joint_pts_closure(test)))
        println()
    end

    collision_nn_file_name = y["collision_nn_file"]

    @save path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name m

    model = BSON.load(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name)[:m]

    xr = rand(6)

    function g(x, m)
        val = Flux.Tracker.data(m(state_to_joint_pts_closure(x))[1])
        return val
    end

    g_closure = (x) -> g(x, model)

    # println(ForwardDiff.gradient(g_closure, xr))

end


run_preprocessing()
