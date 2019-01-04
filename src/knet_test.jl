using Knet
using ForwardDiff
using Calculus

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

function predict(w,x)
    # x = Knet.mat(x)
    for i=1:2:length(w)-2
        x = Knet.relu.(w[i]*x .+ w[i+1])
    end
    return w[end-1]*x .+ w[end]
end

loss(w,x,y) = Knet.mean(abs2, y - predict(w,x))
loss2(w,x,y) = Knet.mean(abs2, y - predict(w,x))[1]
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

path_to_src = Base.source_dir()
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)

println("loaded robot: $loaded_robot")

relaxedIK = get_standard(path_to_src, loaded_robot; preconfigured=true)
cv = c.CollisionVars(path_to_src)


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
num_samples = 20000
ins = []
outs = []
test_ins = []
test_outs = []

for i=1:num_samples
    # in = rand(Uniform(-6,6), num_dof)
    in = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
    # out = [min(c.get_score(in, cv), 1.3)]
    out = [c.get_score(in, cv)]


    push!(ins, state_to_joint_pts_closure(in))
    push!(outs, out)

    println("sample $i of $num_samples ::: state: $in, y: $out")
end


for i=1:100
    # in = rand(Uniform(-6,6), num_dof)
    in = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
    # out = [min(c.get_score(in, cv), 1.3)]
    out = [c.get_score(in, cv)]

    push!(test_ins, state_to_joint_pts_closure(in))
    push!(test_outs, out)
end

################################################################################


# Make batches #################################################################
batch = Knet.minibatch(ins, outs, 500)
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


# Make neural net ##############################################################
net_width = length(ins[1]) + 3
rand_val = 1.0

w = [ rand_val*Knet.xavier(net_width, length(ins[1]) ), zeros(Float64,net_width,1),
      rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
      rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
      rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
      rand_val*Knet.xavier(1, net_width), zeros(Float64,1,1)  ]

################################################################################


# Optimize #####################################################################
o = optimizers(w, Knet.Adam)
tl = total_loss2(w, test_ins, test_outs)
tl_train = total_loss( w, ins, outs )
println("epoch 0 ::: train loss: $tl_train, test loss: $tl")
num_epochs = 20
for epoch=1:num_epochs
    for b = 1:length(batched_data)
        num_batches = length(batched_data)
        train(w, batched_data[b], o)
        print("*")
    end
    # train(w, data, o)
    tl = total_loss2(w, test_ins, test_outs)
    tl_train = total_loss( w, ins, outs )
    println("epoch $epoch of $num_epochs ::: train loss: $tl_train, test loss: $tl")
    # tl = total_loss2(w, test_ins, test_outs)
    # tl_train = total_loss( w, ins[1:50], outs[1:50] )
    # println("epoch $epoch of $num_epochs ::: train loss: $tl_train, test loss: $tl")
end
################################################################################


################################################################################

function f(w, x)
    p = predict(w, x)
    return p[1]
end

m = x -> f(w, x)

for i=1:length(test_ins)
    input = test_ins[i]
    modeled_out =   m( input )
    y_out = test_outs[i]

    println("modeled_out: $modeled_out, y_out: $y_out \n")
end

# save it ######################################################################

fp = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
y = YAML.load(fp)

collision_nn_file_name = y["collision_nn_file"]

@save path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name w

################################################################################
