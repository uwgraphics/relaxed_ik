# note: contraints are of the form f(x) <= 0, so allowable outputs are NEGATIVE (opposite of scipy)

include("gradient.jl")


function get_constraint_closure(func, grad_method, vars)
    func_closure = x->func(x, vars)
    ∇ = get_∇(func_closure, grad_method)

    function inner(x,grad)
        if length(grad) > 0
            g = ∇(x)
            for i = 1:length(grad)
                grad[i] = g[i]
            end
        end
        return func(x, vars)
    end

    return inner
end
