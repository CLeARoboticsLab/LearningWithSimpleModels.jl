# To use with Revise.jl:
# using Revise
# push!(LOAD_PATH, joinpath(pwd(), "experiments\\jetracer\\src"))
# ] activate experiments\\jetracer\\
## First time only:
## dev .
## instantiate
# using JetRacerExperiment

module JetRacerExperiment

using LearningWithSimpleModels
using BSON
using CairoMakie

include("experiment.jl")
include("controller.jl")
include("run.jl")
include("plots.jl")
include("csv_export.jl")

end #module