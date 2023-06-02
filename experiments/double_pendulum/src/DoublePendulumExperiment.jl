# To use with Revise.jl:
# using Revise
# push!(LOAD_PATH, joinpath(pwd(), "experiments\\double_pendulum\\src"))
# ] activate experiments\\double_pendulum\\
## First time only:
## dev .
## instantiate
# using DoublePendulumExperiment

module DoublePendulumExperiment

using LearningWithSimpleModels
using BSON

include("experiment.jl")
include("controller.jl")
include("run.jl")
# include("plots.jl")

end #module