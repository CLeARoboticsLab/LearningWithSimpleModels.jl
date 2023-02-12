module LearningWithSimpleModels

using Flux:
    Dense,
    Chain,
    glorot_uniform,
    zeros32,
    setup,
    Adam,
    withgradient,
    update!
using Distributions: Uniform
using BSON: @save, @load
using GLMakie
import ProgressMeter

include("types.jl")
export DyanmicsParameters, NoDyanmicsParameters, Dynamics,
    ControllerParameters, Controller,
    CostParameters, QuadraticCostParameters, Cost,
    WalkingWindowAlgorithm, RandomInitialAlgorithm,
    TrainingParameters, SimulationParameters,
    Spline, EvaluationData

include("spline.jl")
export evaluate, to_velocity_and_heading_angle, figure_eight

include("dynamics.jl")
include("controller.jl") 

include("cost.jl")
export quadratic_cost

include("model.jl")
include("gradient_estimate.jl")

include("train.jl")
export train

include("plot_utils.jl")
export plot_losses, plot_evaluation

include("evaluate.jl")
export evaluate_model

end
