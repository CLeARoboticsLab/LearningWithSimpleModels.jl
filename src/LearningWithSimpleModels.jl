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
using RosSockets
import ProgressMeter
using Rotations
using LinearAlgebra
import JSON

include("types.jl")
export DyanmicsParameters, NoDyanmicsParameters, Dynamics,
    ControllerParameters, Controller,
    CostParameters, QuadraticCostParameters, Cost,
    WalkingWindowAlgorithm, RandomInitialAlgorithm, HardwareTrainingAlgorithm,
    simulation_timestep, model_call, adam, gradient_descent,
    TrainingParameters, SimulationParameters, EvaluationParameters,
    FigEightCircle, Spline

include("task.jl")
export to_velocity_and_heading_angle, wrapped_time

include("spline.jl")
export figure_eight

include("fig_eight_circle.jl")
export evaluate

include("spline_segment.jl")
include("dynamics.jl")
include("controller.jl")
include("cost.jl")
include("model.jl")
include("gradient_estimate.jl")
include("communication.jl")
include("policy_update.jl")
include("train.jl")
export train

include("plot_utils.jl")
export plot_hardware_evaluation

include("evaluate.jl")
export evaluate_model, evaluate_on_hardware

end
