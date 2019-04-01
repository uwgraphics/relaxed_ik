

function obj_master(x, grad, vars)
    if length(grad) > 0
        g = zeros(length(grad))
        for i in 1:length(vars.∇s)
            g += vars.weight_priors[i]*vars.∇s[i](x)
        end
        for i = 1:length(grad)
            grad[i] = g[i]
        end
    end

    sum = 0.0
    for i in 1:length(vars.objective_closures)
        sum += vars.weight_priors[i]*vars.objective_closures[i](x)
    end

    return sum
end


function get_obj_closure(func, vars)
    # takes in an objective function that is dependent on x as well as a vars object, and returns a function that
    #   is only dependent on x
    return x -> func(x, vars)
end
