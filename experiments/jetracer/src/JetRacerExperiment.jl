# To use with Revise.jl:
# using Revise
# push!(LOAD_PATH, joinpath(pwd(), "experiments\\jetracer\\src"))
# ] activate experiments\\jetracer\\
# using JetRacerExperiment

module JetRacerExperiment

using LearningWithSimpleModels
using RosSockets
import JSON
using Flux: Chain, withgradient, setup, Descent, Adam, update!
using Rotations
using LinearAlgebra
import ProgressMeter
using CairoMakie
using BSON: @load, @save

include("experiment.jl")
include("controller.jl")
include("communication.jl")
include("dynamics.jl")
include("train.jl")
include("gradient_estimate.jl")
include("plot_utils.jl")
include("evaluate.jl")
include("tests.jl")
include("run.jl")

end #module