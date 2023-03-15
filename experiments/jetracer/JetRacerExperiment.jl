# To use with Revise.jl:
# using Revise
# push!(LOAD_PATH, joinpath(pwd(), "experiments\\jetracer\\"))
# using JetRacerExperiment

module JetRacerExperiment

using LearningWithSimpleModels
using RosSockets #TODO should use a separate Project.toml for these deps
import JSON

include("experiment.jl")

end #module