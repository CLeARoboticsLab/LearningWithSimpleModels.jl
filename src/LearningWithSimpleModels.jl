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
using BSON
using LinearAlgebra: diagm
using CairoMakie
using RosSockets
import ProgressMeter
using Rotations
using LinearAlgebra
import JSON
using LaTeXStrings
using Statistics: mean, std
using CSV
using Tables
using Printf: @sprintf

include("types.jl")
export DyanmicsParameters, NoDyanmicsParameters, Dynamics,
    ControllerParameters, Controller,
    CostParameters, QuadraticCostParameters, Cost,
    WalkingWindowAlgorithm, RandomInitialAlgorithm, HardwareTrainingAlgorithm,
    simulation_timestep, model_call, adam, gradient_descent,
    TrainingParameters, SimulationParameters, EvaluationParameters,
    QuadraticSpline, CubicSpline, NoSpline,
    UnicycleEvalType, DoublePendulumEvalType,
    ConstantTask, FigEightCircle, Spline,
    RolloutData, TrainingData

include("task.jl")
export to_velocity_and_heading_angle, wrapped_time, eval_all, end_effector_position

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
export plot_hardware_evaluation, multi_training_plot, final_eval_plot,
    final_model_outputs_plot, animate_final_evaluation, plot_variances

include("evaluate.jl")
export evaluate_model, evaluate_on_hardware

include("csv.jl")
export make_csv

end
