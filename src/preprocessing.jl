ENV["PYTHON"] = "/usr/bin/python"

using PyCall, Flux, YAML,BSON, Calculus, ForwardDiff, CuArrays
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/Utils_Julia/transformations.jl")

@pyimport RelaxedIK.Utils.collision_transfer as c


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

path_to_src = Base.source_dir()
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)

relaxedIK = get_standard(path_to_src, loaded_robot)
cv = c.CollisionVars(path_to_src)

num_dof = relaxedIK.relaxedIK_vars.robot.num_dof

state_to_joint_pts_closure = (x) -> state_to_joint_pts(x, relaxedIK.relaxedIK_vars)

ins = []
outs = []

num_samples = 200000

# make samples
last_state = relaxedIK.relaxedIK_vars.vars.init_state
for i=1:num_samples
    global last_state
    vel = 0.02 * rand(num_dof)
    in = last_state + vel
    last_state = in
    out = c.get_score(in, cv)

    push!(ins, state_to_joint_pts_closure(in))
    push!(outs, [out])

    println("sample $i of $num_samples")
end


data = zip(ins, outs)


m = Chain(
    Dense(length(state_to_joint_pts_closure(rand(num_dof))), 50),
    Dense(50, 30),
    Dense(30, 30),
    Dense(30, 30),
    Dense(30, 30),
    Dense(30, 1)
)

p = Flux.params(m)
opt = Flux.Optimise.ADAM(p)
loss(x, y) = Flux.mse(m(x), y)

@epochs 30 Flux.train!( loss, data, opt )

f = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
y = YAML.load(f)

collision_nn_file_name = y["collision_nn_file"]

@save path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name m


for i =1:10
    test = rand(num_dof)
    println(c.get_score(test, cv))
    println(m(state_to_joint_pts_closure(test)))
    println()
end
