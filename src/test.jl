#!/usr/bin/env julia

using YAML


path_to_src = Base.source_dir()

f = open(path_to_src * "/RelaxedIK/Config/info_files/ur5_info.yaml")

y = YAML.load(f)

# println(y)

# include("RelaxedIK/GROOVE_Julia/objective.jl")
# include("RelaxedIK/Spacetime_Julia/arm.jl")
# include("RelaxedIK/Spacetime_Julia/robot.jl")
