# To use with Revise.jl:
# using Revise
# push!(LOAD_PATH, joinpath(pwd(), "experiments\\jetracer\\src"))
# ] activate experiments\\jetracer\\
# using JetRacerExperiment

module JetRacerExperiment

using LearningWithSimpleModels
using RosSockets
import JSON

include("experiment.jl")
include("tests.jl")

end #module