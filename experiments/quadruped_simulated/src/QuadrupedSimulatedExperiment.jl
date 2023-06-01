# To use with Revise.jl:
# using Revise
# push!(LOAD_PATH, joinpath(pwd(), "experiments\\quadruped_simulated\\src"))
# ] activate experiments\\quadruped_simulated\\
## First time only:
## dev .
## instantiate
# using QuadrupedSimulatedExperiment

module QuadrupedSimulatedExperiment

using LearningWithSimpleModels

include("experiment.jl")
include("controller.jl")
include("run.jl")

end