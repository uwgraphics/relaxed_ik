include("RelaxedIK/Utils_Julia/autoparams.jl")
include("RelaxedIK/Utils_Julia/collision_utils.jl")
using BSON
@pyimport RelaxedIK.Utils.collision_transfer as c


function calibrate_nns(path_to_src)
    loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
    loaded_robot = readline(loaded_robot_file)
    fp = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
    y = YAML.load(fp)

    cv = c.CollisionVars(path_to_src)
    relaxedIK = get_standard(path_to_src, loaded_robot; preconfigured=true)

    collision_nn_file_name = y["collision_nn_file"]
    w = BSON.load(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name)[:w]
    model = (x) -> predict(w, x)[1]
    w2 = BSON.load(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_2")[:w2]
    model2 = (x) -> predict(w2, x)[1]
    w3 = BSON.load(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_3")[:w3]
    model3 = (x) -> predict(w3, x)[1]

    ################################################################################

    # get t, c, and f values #######################################################
    t_val, c_val, f_val = get_t_c_and_f_values(w, cv, relaxedIK)
    println("calibrated network one.")
    t_val2, c_val2, f_val2 = get_t_c_and_f_values(w2, cv, relaxedIK)
    println("calibrated network two.")
    t_val3, c_val3, f_val3 = get_t_c_and_f_values(w3, cv, relaxedIK)
    println("calibrated network three.")

    model1_acc = 0.0
    model2_acc = 0.0
    model3_acc = 0.0
    total_acc_count = 2000

    for i = 1:total_acc_count
        r = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
        state = state_to_joint_pts_withreturn(r, relaxedIK.relaxedIK_vars)
        model1_score = model(state)
        model2_score = model2(state)
        model3_score = model3(state)
        ground_truth_score = c.get_score(r, cv)
        thresh = 1.0
        if ground_truth_score >= 5.0
            if model1_score >= thresh
                model1_acc += 1.
            end

            if model2_score >= thresh
                model2_acc += 1.
            end

            if model3_score >= thresh
                model3_acc += 1.
            end
        else
            if model1_score < thresh
                model1_acc += 1.
            end

            if model2_score < thresh
                model2_acc += 1.
            end

            if model3_score < thresh
                model3_acc += 1.
            end
        end
    end

    model1_acc = model1_acc/total_acc_count
    model2_acc = model2_acc/total_acc_count
    model3_acc = model3_acc/total_acc_count

    # maxidx = argmax([model1_acc, model2_acc, model3_acc])
    # minidx = argmax([model1_acc, model2_acc, model3_acc])
    sorted = sortperm([model1_acc, model2_acc, model3_acc])
    println("model 1 accuracy: $model1_acc, model 2 accuracy: $model2_acc, model 3 accuracy: $model3_acc")

    fp = open(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_params", "w")
    write(fp, "$t_val, $c_val, $f_val")
    close(fp)

    fp = open(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_params_2", "w")
    write(fp, "$t_val2, $c_val2, $f_val2")
    close(fp)

    fp = open(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_params_3", "w")
    write(fp, "$t_val3, $c_val3, $f_val3")
    close(fp)

    fp = open(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_network_rank", "w")
    write(fp, "$(sorted[3]), $(sorted[2]), $(sorted[1])")
    close(fp)
end

# path_to_src = Base.source_dir()
# calibrate_nns(path_to_src)
