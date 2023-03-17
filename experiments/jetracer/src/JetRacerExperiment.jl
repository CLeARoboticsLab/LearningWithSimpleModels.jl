# To use with Revise.jl:
# using Revise
# push!(LOAD_PATH, joinpath(pwd(), "experiments\\jetracer\\src"))
# ] activate experiments\\jetracer\\
# using JetRacerExperiment

module JetRacerExperiment

using LearningWithSimpleModels
using RosSockets
import JSON
using Flux: Chain, withgradient
using Rotations
using LinearAlgebra
import ProgressMeter

include("experiment.jl")
include("controller.jl")
include("communication.jl")
include("dynamics.jl")
include("gradient_estimate.jl")
include("tests.jl")
include("run.jl")

end #module