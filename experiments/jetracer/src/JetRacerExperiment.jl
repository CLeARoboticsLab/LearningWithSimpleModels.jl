# To use with Revise.jl:
# using Revise
# push!(LOAD_PATH, joinpath(pwd(), "experiments\\jetracer\\src"))
# ] activate experiments\\jetracer\\
# using JetRacerExperiment

module JetRacerExperiment

using LearningWithSimpleModels
using RosSockets
import JSON
using Flux: Chain

include("experiment.jl")
include("communication.jl")
include("dynamics.jl")
include("tests.jl")

end #module