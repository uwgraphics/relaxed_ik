using Flux
using Knet
using BenchmarkTools

function predict(w,x)
    for i=1:2:length(w)-2
        x = Knet.relu.(w[i]*x .+ w[i+1])
    end
    return w[end-1]*x .+ w[end]
end

net_width = 30
rand_val = 1.0
w = [ rand_val*Knet.xavier(net_width, 24 ), zeros(Float64,net_width,1),
      rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
      rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
      rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
      rand_val*Knet.xavier(1, net_width), zeros(Float64,1,1)  ]

nn_val = 30
m = Chain( Dense(24, nn_val, Flux.relu),
    Dense(nn_val, nn_val, Flux.relu),
    Dense(nn_val, nn_val, Flux.relu),
    Dense(nn_val, nn_val, Flux.relu),
    Dense(nn_val, 1)
)

x = rand(24)

@btime m(x)
@btime predict(w, x)
