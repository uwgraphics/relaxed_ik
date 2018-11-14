
using ForwardDiff


indices = [1,3,4]

function t(x)
    subchain = []
    for i=1:length(indices)
        push!(subchain, x[indices[i]])
    end
    return subchain[1]^2
end

println(ForwardDiff.gradient(t, [1.,2.,3.,4.,5.]))
