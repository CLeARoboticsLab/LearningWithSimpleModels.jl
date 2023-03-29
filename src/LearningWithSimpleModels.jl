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
    TrainingAlgorithm, WalkingWindowAlgorithm, RandomInitialAlgorithm,
    simulation_timestep, model_call, adam, gradient_descent,
    TrainingParameters, SimulationParameters, EvaluationParameters,
    AbstractTask, FigEightCircle, Spline, RolloutData, TrainingData,EvaluationData

include("spline.jl")
export evaluate, to_velocity_and_heading_angle, figure_eight, 
    end_time, spline_segment, eval_all, evaluate_segment

include("fig_eight_circle.jl")
export wrapped_time

include("dynamics.jl")
export f_simple

include("controller.jl")
export next_command

include("cost.jl")
export quadratic_cost, stage_cost

include("model.jl")
export make_model, call_model

include("gradient_estimate.jl")

include("train.jl")
export train, save_model, save_all_data

include("plot_utils.jl")
export plot_losses, animate_training, plot_evaluation, animate_evaluation

include("evaluate.jl")
export evaluate_model

end
