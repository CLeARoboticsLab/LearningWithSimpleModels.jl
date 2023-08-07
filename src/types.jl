abstract type DyanmicsParameters end
struct NoDyanmicsParameters <: DyanmicsParameters end

Base.@kwdef struct Dynamics
    params::DyanmicsParameters = NoDyanmicsParameters()
    f::Function
end

abstract type ControllerParameters end

Base.@kwdef struct Controller
    params::ControllerParameters
    policy::Function
end

abstract type CostParameters end

Base.@kwdef struct QuadraticCostParameters <: CostParameters
    Q::Matrix{Float64}
    R::Matrix{Float64}
end

Base.@kwdef struct Cost
    params::CostParameters
    g::Function
end

abstract type TrainingAlgorithm end

Base.@kwdef struct WalkingWindowAlgorithm <:TrainingAlgorithm
    segs_per_rollout = 60
    segs_in_window::Integer = 5
    n_beginning_segs_to_truncate::Integer = 0
end
Base.show(io::IO, p::WalkingWindowAlgorithm) = print(io,
    "Walking Window Algorithm
    Segments per rollout: $(p.segs_per_rollout)
    Segments in window: $(p.segs_in_window)
    Skipped beginning segments: $(p.n_beginning_segs_to_truncate)"
)

Base.@kwdef struct RandomInitialAlgorithm <:TrainingAlgorithm 
    variances::Vector{Float64}
    perc_of_task_to_sample::Real = 1.0
    n_rollouts_per_update::Integer = 1
    n_beginning_segs_to_truncate::Integer = 0
    segs_per_rollout::Integer = 20
    segs_in_window::Integer = 5
    to_state::Function
    task_time_est::Union{Function, Nothing} = nothing
end
Base.show(io::IO, p::RandomInitialAlgorithm) = print(io,
    "Random Initial Algorithm:
    Num of rollouts per update: $(p.n_rollouts_per_update)
    Num segments per rollout: $(p.segs_per_rollout)
    Skipped beginning segments: $(p.n_beginning_segs_to_truncate)
    Segments in window: $(p.segs_in_window)
    Standard Deviations:
    $(round.(sqrt.(p.variances); digits=4))"
)

Base.@kwdef struct HardwareTrainingAlgorithm <:TrainingAlgorithm
    seconds_per_rollout::Float64 = 12.0
    n_beginning_segs_to_truncate::Integer = 0
    use_window::Bool = false
    segs_in_window::Integer = 10
    stopping_segments = 0
    task_time_est::Union{Function, Nothing} = nothing
end
Base.show(io::IO, p::HardwareTrainingAlgorithm) = print(io,
    "Hardware Training Algorithm
    Seconds per rollout: $(p.seconds_per_rollout)
    Skipped beginning segments: $(p.n_beginning_segs_to_truncate)
    Use window: $(p.use_window)
    Segments in window: $(p.segs_in_window)"
)

@enum Optim adam gradient_descent
@enum LossAgg simulation_timestep model_call

Base.@kwdef struct TrainingParameters
    name::String = "experiment"
    save_path = ".data"
    hidden_layer_sizes::Vector{<:Integer} = [64, 64]
    learning_rate::Float64 = 1e-3
    iters::Integer = 50
    optim::Optim = adam
    loss_aggregation::LossAgg = simulation_timestep
    plot::Bool = true
    save_model::Bool = true
    save_plot::Bool = true
    save_animation::Bool = false
    save_all_data::Bool = true
end
Base.show(io::IO, p::TrainingParameters) = print(io,
    "Training Parameters: 
    Hidden layer sizes: $(p.hidden_layer_sizes) 
    Learning rate: $(p.learning_rate) 
    Iterations: $(p.iters)
    Optimizer: $(p.optim)
    Loss Aggregation: $(p.loss_aggregation)"
)

abstract type SplineSegmentType end
struct QuadraticSpline <: SplineSegmentType end
struct CubicSpline <: SplineSegmentType end
struct NoSpline <: SplineSegmentType end

function default_model_input_function(
    t::Real,
    task_time::Real,
    x::Vector{<:Real},
)
    t_transformed = [cos(2*π*t/task_time), sin(2*π*t/task_time)]
    x_transformed = [
        x[1],
        x[2],
        x[3]*cos(x[4]),
        x[3]*sin(x[4])
    ]
    return vcat(x_transformed, t_transformed)
end

Base.@kwdef struct SimulationParameters
    x0::Vector{Float64}
    n_inputs::Integer
    dt::Float64 = 0.01
    model_dt::Float64 = 0.5
    model_scale::Vector{Float64} = ones(8)
    model_in_dim::Integer = 6
    model_input_function::Function = default_model_input_function
    spline_seg_type::SplineSegmentType = QuadraticSpline()
end
Base.show(io::IO, p::SimulationParameters) = print(io,
    "Simulation Parameters: 
    Initial state: $(p.x0)
    Simulation dt: $(p.dt) 
    Time between model calls: $(p.model_dt)
    Model scale: $(p.model_scale)"
)

abstract type AbstractEvalType end
struct UnicycleEvalType <: AbstractEvalType end
struct DoublePendulumEvalType <: AbstractEvalType end

Base.@kwdef struct EvaluationParameters
    name::String = "experiment"
    type::AbstractEvalType = UnicycleEvalType()
    f::Union{Nothing, Function} = nothing
    path = ".data"
    n_task_executions::Integer = 1
    save_plot::Bool = true
    save_animation::Bool = false
end

abstract type AbstractTask end

struct Spline <: AbstractTask
    ts::Vector{Float64}
    coeffs_x::Vector{Float64}
    coeffs_y::Vector{Float64}
    x0::Float64
    y0::Float64
end

struct FigEightCircle <: AbstractTask
    r::Real
    time::Real
    v::Real
end

struct ConstantTask <: AbstractTask
    xd::Vector{Float64}
    time::Real
end

struct Connections
    feedback::Connection
    control::Connection
    timing::Connection
    rollout::Connection
end

Base.@kwdef struct RolloutData
    ts::Vector{Float64}
    xs::Matrix{Float64}
    us::Matrix{Float64}
    task_t0::Float64                # time along task where this rollout starts
    idx_segs::Vector{<:Integer}     # overall index at each model call
    t0_segs::Vector{Float64}        # time at each model call 
    x0_segs::Matrix{Float64}        # state at each model call
    setpoints::Matrix{Float64}      # Corrected setpoints, from calling model
    gain_adjs::Matrix{Float64}      # Gain adjustments, from calling model
    ctrl_setpoints::Matrix{Float64} # setpoints input into controller
    xf::Vector{Float64}             # final state, at T+1
    loss::Float64
end

Base.@kwdef struct TrainingData
    losses::Vector{Float64}
    rollouts::Vector{RolloutData}
end

Base.@kwdef struct EvaluationData
    r::RolloutData
    r_no_model::RolloutData         # rollout data w/o using the model       
    xs_task::Vector{Float64}        # x-coords of task
    ys_task::Vector{Float64}        # y-coords of task
end
