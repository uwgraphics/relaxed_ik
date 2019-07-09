using ForwardDiff
using Calculus
using ReverseDiff

function get_∇(func, grad_method)
    # func needs to already be a function just with respect to x
    if grad_method == "forward_ad"
        return x->ForwardDiff.gradient(func, x)
    elseif grad_method == "reverse_ad"
        return x->ReverseDiff.gradient(func, x)
    elseif grad_method == "finite_diff"
        return x->Calculus.gradient(func, x)
    else
        return x->Calculus.gradient(func, x) # default to finite differencing
    end
end
