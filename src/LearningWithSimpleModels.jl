module LearningWithSimpleModels

using Flux:
    Dense,
    Chain,
    glorot_uniform,
    zeros32,
    setup,
    Adam,
    withgradient,
    update!,
    Descent,
    f64
using Distributions: Uniform, MvNormal
using BSON: @save, @load
using LinearAlgebra: diagm
using GLMakie
import ProgressMeter

include("types.jl")
export DyanmicsParameters, NoDyanmicsParameters, Dynamics,
    ControllerParameters, Controller,
    CostParameters, QuadraticCostParameters, Cost,
    WalkingWindowAlgorithm, RandomInitialAlgorithm,
    simulation_timestep, model_call, adam, gradient_descent,
    TrainingParameters, SimulationParameters, EvaluationParameters,
    Spline, RolloutData, EvaluationData

include("spline.jl")
export evaluate, to_velocity_and_heading_angle, figure_eight, end_time, spline_segment

include("dynamics.jl")
include("controller.jl") 

include("cost.jl")
export quadratic_cost

include("model.jl")
export make_model, call_model

include("gradient_estimate.jl")

include("train.jl")
export train

include("plot_utils.jl")

include("evaluate.jl")
export evaluate_model

end
