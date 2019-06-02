#!/usr/bin/env julia

using Knet
using ForwardDiff
using Calculus
using Random
using RobotOS

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
include("RelaxedIK/Utils_Julia/collision_utils.jl")
include("calibrate_nns.jl")
@pyimport RelaxedIK.Utils.collision_transfer as c

@rosimport std_msgs.msg: Bool

rostypegen()
using .std_msgs.msg



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

function predict(w,x)
    # x = Knet.mat(x)
    for i=1:2:length(w)-2
        x = Knet.relu.(w[i]*x .+ w[i+1])
    end
    return w[end-1]*x .+ w[end]
end

loss(w,x,y) = Knet.mean(abs2, y - predict(w,x) )
loss2(w,x,y) = Knet.mean(abs2, y - predict(w,x))[1]
# loss(w,x,y) = (y[1] - predict(w,x)[1])^2
# loss2(w,x,y) = ((y[1] - predict(w,x)[1])^2)[1]
lossgradient = grad(loss)

function total_loss(w, all_x, all_y)
    sum = 0.0
    for i = 1:length(all_x)
        sum += loss(w, all_x[i], all_y[i])
    end
    return sum
end

function total_loss2(w, all_x, all_y)
    sum = 0.0
    for i = 1:length(all_x)
        sum += loss2(w, all_x[i], all_y[i])
    end
    return sum
end

function train(model, data, optim)
    for (x,y) in data
        grads = lossgradient(model,x,y)
        Knet.update!(model, grads, optim)
    end
end

function get_rand_state_with_bounds(bounds)
    sample = []
    for b in bounds
        push!(sample, rand(Uniform(b[1], b[2])))
    end
    return sample
end

function shuffle_ins_and_outs(ins, outs, states)
    new_ins = []
    new_outs = []
    new_states = []

    idxs = 1:length(ins)
    shuffled_idxs = shuffle(idxs)

    for i=1:length(shuffled_idxs)
        push!(new_ins, ins[shuffled_idxs[i]])
        push!(new_outs, outs[shuffled_idxs[i]])
        push!(new_states, states[shuffled_idxs[i]])
    end

    return new_ins, new_outs, new_states
end

function get_batched_data(ins, outs, batch_size)
    batch = Knet.minibatch(ins, outs, batch_size)
    batches = []

    for (index, value) in enumerate(batch)
        push!(batches, value)
    end

    batched_data = []
    for batch_idx = 1:length(batches)
        push!(batched_data, [])
        for i = 1:length(batches[batch_idx][1])
            in = batches[batch_idx][1][i]
            out = batches[batch_idx][2][i]
            push!( batched_data[batch_idx], (in, out) )
        end
    end

    return batched_data
end

quit = false
function quit_cb(data::BoolMsg)
    global quit
    quit = data.data
end

path_to_src = Base.source_dir()
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)

println("loaded robot: $loaded_robot")

relaxedIK = get_standard(path_to_src, loaded_robot; preconfigured=true)
cv = c.CollisionVars(path_to_src)

# init_node("preprocessing")

Subscriber{BoolMsg}("/relaxed_ik/quit", quit_cb, queue_size=3)
0
sleep(0.3)

num_dof = relaxedIK.relaxedIK_vars.robot.num_dof

state_to_joint_pts_closure = (x) -> state_to_joint_pts(x, relaxedIK.relaxedIK_vars)

##############################################################################################################################################################
##############################################################################################################################################################
##############################################################################################################################################################
##############################################################################################################################################################
##############################################################################################################################################################
##############################################################################################################################################################
##############################################################################################################################################################


# Create data ##################################################################
num_samples = 200000
ins = []
outs = []
states = []
test_ins = []
test_outs = []
test_states = []

for i=1:num_samples
    # in = rand(Uniform(-6,6), num_dof)
    in = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
    # out = [min(c.get_score(in, cv), 2.0)]
    out = [c.get_score(in, cv)]
    #score = c.get_score(in, cv)
    #if score > 0.095
    #    out = [1.0]
    #else
    #    out = [-1.0]
    #end

    push!(states, in)
    push!(ins, state_to_joint_pts_closure(in))
    push!(outs, out)

    global quit
    if quit == true
        quit = false
        break
    end

    println("sample $i of $num_samples ::: state: $in, y: $out")
end


# add manually specified training examples
fp = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
y = YAML.load(fp)
close(fp)

collision_file_name = y["collision_file_name"]

fp = open(path_to_src * "/RelaxedIK/Config/collision_files/" * collision_file_name)
y = YAML.load(fp)


training_states = y["training_states"]
if ! (training_states == nothing)
    num_samples = length(training_states)
    for i=1:num_samples
        # in = rand(Uniform(-6,6), num_dof)
        # in = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
        in = training_states[i]
        # out = [min(c.get_score(in, cv), 2.0)]
        out = [c.get_score(in, cv)]
        # score = c.get_score(in, cv)
        #if score > 0.095
        #    out = [1.0]
        #else
        #    out = [-1.0]
        #end

        push!(states, in)
        push!(ins, state_to_joint_pts_closure(in))
        push!(outs, out)

        println("manual sample $i of $num_samples ::: state: $in, y: $out")
    end
end

problem_states = y["problem_states"]
if ! (problem_states == nothing)
    num_samples = length(problem_states)
    length_of_sample = length(problem_states[1])
    num_rands_per = 50
    for i = 1:num_samples
        for j = 1:num_rands_per
            r = rand(Uniform(-.005,.005), length_of_sample)
            in = problem_states[i] + r
            out = [c.get_score(in ,cv)]
            #score = c.get_score(in, cv)
            #if score > 0.095
            #    out = [1.0]
            #else
            #    out = [-1.0]
            #end

            push!(states, in)
            push!(ins, state_to_joint_pts_closure(in))
            push!(outs, out)

            println("problem state sample $i ($j / $num_rands_per) of $num_samples ::: state: $in, y: $out")
        end
    end
end

close(fp)


for i=1:50
    # in = rand(Uniform(-6,6), num_dof)
    in = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
    # out = [min(c.get_score(in, cv), 2.0)]
    out = [c.get_score(in, cv)]
    #score = c.get_score(in, cv)
    #if score > 0.095
    #    out = [1.0]
    #else
    #    out = [-1.0]
    #end

    push!(test_states, in)
    push!(test_ins, state_to_joint_pts_closure(in))
    push!(test_outs, out)
end

################################################################################


# Make batches #################################################################
batch = Knet.minibatch(ins, outs, 200)
batches = []

for (index, value) in enumerate(batch)
    push!(batches, value)
end

################################################################################


# Finalize data ################################################################
batched_data = []
data = []
test_data = []

for batch_idx = 1:length(batches)
    push!(batched_data, [])
    for i = 1:length(batches[batch_idx][1])
        in = batches[batch_idx][1][i]
        out = batches[batch_idx][2][i]
        push!( batched_data[batch_idx], (in, out) )
    end
end

for i = 1:length(ins)
    push!( data, (ins[i], outs[i]) )
end

for i = 1:length(test_ins)
    push!( test_data, (test_ins[i], test_outs[i]) )
end

################################################################################

#println(new_outs[1])
#println(c.get_score(new_states[1], cv))

# Make neural net ##############################################################
# net_width = length(ins[1]) + 8
# net_width = length(ins[1])
net_width = 18
rand_val = 1.0
w = [ rand_val*Knet.xavier(net_width, length(ins[1]) ), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(1, net_width), zeros(Float64,1,1)  ]


net_width = 26
rand_val = 1.0
w2 = [ rand_val*Knet.xavier(net_width, length(ins[1]) ), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(1, net_width), zeros(Float64,1,1)  ]


net_width = 34
rand_val = 1.0
w3 = [ rand_val*Knet.xavier(net_width, length(ins[1]) ), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
    rand_val*Knet.xavier(1, net_width), zeros(Float64,1,1)  ]
################################################################################


# Optimize #####################################################################
o = optimizers(w, Knet.Adam)
o2 = optimizers(w2, Knet.Adam)
o3 = optimizers(w3, Knet.Adam)

tl = total_loss2(w, test_ins, test_outs)
tl_train = total_loss( w, ins, outs )
println("epoch 0 ::: train loss: $tl_train, test loss: $tl")
num_epochs = 2
batch_size = 200
best_w = []
best_score = 10000000000000000.0
improve_idx = 1

for epoch=1:num_epochs
    # shuffle data here...get new batched data
    new_ins, new_outs, new_states = shuffle_ins_and_outs(ins, outs, states)
    batched_data = get_batched_data(new_ins, new_outs, batch_size)

    # global quit
    #if quit == true
    #    global w
    #    w = copy(best_w)
    #    println("quitting training.")
    #    break
    # end

    for b = 1:length(batched_data)
        num_batches = length(batched_data)
        global w, w2, w3
        train(w, batched_data[b], o)
        train(w2, batched_data[b], o2)
        train(w3, batched_data[b], o3)
        print("*")

        #=
        tl = total_loss2(w, test_ins, test_outs)
        tl_train = total_loss( w, new_ins[1:200], new_outs[1:200] )
        global best_w
        global best_score
        if tl_train < best_score
            println("improved best score")
            best_w = copy(w)
            best_score = tl_train
            global improve_idx
            improve_idx = 1
        else
            global improve_idx
            improve_idx += 1
            max_iters = 500
            w = best_w
            if improve_idx > max_iters
                println("have not improved in $max_iters iterations.  quitting.")
                global quit
                quit = true
                break
            end
            =#
        # end

        # global quit
        #if quit == true
        #    global w
        #    w = copy(best_w)
        #    break
        # end
    end
    num_batches = length(batched_data)
    tl = total_loss2(w, test_ins, test_outs)
    tl_train = total_loss( w, new_ins, new_outs )
    # println("epoch $epoch of $num_epochs, $b of $num_batches ::: train loss: $tl_train, test loss: $tl")
    println("\nepoch $epoch of $num_epochs ::: train loss: $tl_train, test loss: $tl")
end

# w = copy(best_w)
################################################################################


################################################################################

function f(w, x)
    p = predict(w, x)
    return p[1]
end

m = x -> f(w, x)

# for i=1:length(test_ins)
#    input = test_ins[i]
#    modeled_out =   m( input )
#    y_out = test_outs[i]
#
#    println("modeled_out: $modeled_out, y_out: $y_out \n")
# end

# save it ######################################################################

fp = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
y = YAML.load(fp)

collision_nn_file_name = y["collision_nn_file"]

@save path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name w
@save path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_2" w2
@save path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_3" w3

################################################################################


println("Preprocessing almost complete.  Calculating calibration parameters...")
calibrate_nns(path_to_src)
#=
# get t, c, and f values #######################################################
t_val, c_val, f_val = get_t_c_and_f_values(w, cv, relaxedIK)
t_val2, c_val2, f_val2 = get_t_c_and_f_values(w2, cv, relaxedIK)
t_val3, c_val3, f_val3 = get_t_c_and_f_values(w3, cv, relaxedIK)


fp = open(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_params", "w")
write(fp, "$t_val, $c_val, $f_val")
close(fp)

fp = open(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_params_2", "w")
write(fp, "$t_val2, $c_val2, $f_val2")
close(fp)

fp = open(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_params_3", "w")
write(fp, "$t_val3, $c_val3, $f_val3")
close(fp)
=#

################################################################################
