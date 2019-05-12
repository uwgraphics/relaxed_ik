include("autoparams.jl")
include("../relaxedIK.jl")
include("nn_utils.jl")
using PyCall
@pyimport RelaxedIK.Utils.collision_transfer as c
using BSON
import Distributions: Uniform
using Statistics


function get_rand_state_with_bounds(bounds)
    sample = []
    for b in bounds
        push!(sample, rand(Uniform(b[1], b[2])))
    end
    return sample
end


function get_t_c_and_f_values(w, collision_check_py_object, relaxedIK)
    model_scores = []
    ground_truth_scores = []

    nn_model = (x) -> predict(w, x)[1]

    for i = 1:7000
        r = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
        state = state_to_joint_pts_withreturn(r, relaxedIK.relaxedIK_vars)
        push!(model_scores, nn_model(state))
        push!(ground_truth_scores, c.get_score(r, collision_check_py_object))
    end

    collision_vals = []
    min_vals = []
    for i = 1:length(model_scores)
        if abs(ground_truth_scores[i] - 5.0) <= 0.5
            push!(collision_vals, model_scores[i])
        end

        if ground_truth_scores[i] < 0.00000000001
            push!(min_vals, model_scores[i])
        end
    end

    collision_val = Statistics.mean(collision_vals)
    min_val = ( Statistics.mean(min_vals) + minimum(model_scores) ) / 2.0

    split_val = (min_val + collision_val) / 2.0

    t_val = minimum(min_val)
    c_val = get_c_value(split_val, minimum(model_scores))
    f_val = get_f_value(split_val, minimum(model_scores))

    return t_val, c_val, f_val
end

#path_to_src = "/home/rakita/catkin_ws/src/relaxed_ik/src/"
#relaxedIK = get_standard(path_to_src, "sawyer_info.yaml")
#println(relaxedIK.relaxedIK_vars.nn_f)

#collision_nn_file_name = "sawyer_nn"
#w = BSON.load(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name)[:w]
#model = (x) -> predict(w, x)[1]
#cv = c.CollisionVars(path_to_src)

# println(get_t_c_and_f_values(w, cv, relaxedIK))

#=
r = rand(6)
state = state_to_joint_pts_withreturn(r, relaxedIK.relaxedIK_vars)

model_scores = []
ground_truth_scores = []

for i = 1:2000
    r = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
    state = state_to_joint_pts_withreturn(r, relaxedIK.relaxedIK_vars)
    push!(model_scores, model(state))
    push!(ground_truth_scores, c.get_score(r, cv))
end

coll_vals = []
min_vals = []
for i = 1:length(model_scores)
    if abs(ground_truth_scores[i] - 5.0) <= 0.5
        push!(coll_vals, model_scores[i])
    end

    if ground_truth_scores[i] < 0.00000001
        push!(min_vals, model_scores[i])
    end
end

m = ( Statistics.mean(min_vals) + minimum(model_scores) ) / 2.0
println(m)
println(Statistics.mean(coll_vals))
println( (Statistics.mean(coll_vals) + m) / 2.0)
=#
