# To use with Revise.jl:
# using Revise
# push!(LOAD_PATH, joinpath(pwd(), "experiments\\quadruped\\src"))
# ] activate experiments\\quadruped\\
## First time only:
## dev .
## instantiate
# using QuadrupedExperiment

module QuadrupedExperiment

using LearningWithSimpleModels

include("experiment.jl")
include("controller.jl")
include("run.jl")

end #module