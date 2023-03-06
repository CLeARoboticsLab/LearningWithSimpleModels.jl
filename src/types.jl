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
end
Base.show(io::IO, ::WalkingWindowAlgorithm) = print(io,
    "Walking Window Algorithm
    Segments per rollout: $(p.segs_per_rollout)
    Segments in window: $(p.segs_in_window)"
)

Base.@kwdef struct RandomInitialAlgorithm <:TrainingAlgorithm 
    variances::Vector{Float64}
    n_rollouts_per_update::Integer = 1
    segs_per_rollout::Integer = 20
    segs_in_window::Integer = 5
    to_state::Function
end
Base.show(io::IO, p::RandomInitialAlgorithm) = print(io,
    "Random Initial Algorithm:
    Num of rollouts per update: $(p.n_rollouts_per_update)
    Num segments per rollout: $(p.segs_per_rollout)
    Segments in window: $(p.segs_in_window)
    Standard Deviations:
    $(round.(sqrt.(p.variances); digits=4))"
)

#TODO: make LossAggregationStyle an enum?
abstract type LossAggregationStyle end
struct AtModelCall <: LossAggregationStyle end
struct AtSimulationTimestep <: LossAggregationStyle end

@enum Optim adam gradient_descent

Base.@kwdef struct TrainingParameters
    hidden_layer_sizes::Vector{<:Integer} = [64, 64]
    learning_rate::Float64 = 1e-3
    iters::Integer = 50
    optim::Optim = adam
    loss_aggregation::LossAggregationStyle = AtSimulationTimestep()
    save_path = nothing
    plot_save_path = nothing
end
Base.show(io::IO, p::TrainingParameters) = print(io,
    "Training Parameters: 
    Hidden layer sizes: $(p.hidden_layer_sizes) 
    Learning rate: $(p.learning_rate) 
    Iterations: $(p.iters)
    Optimizer: $(p.optim)
    Loss Aggregation: $(p.loss_aggregation)"
)

Base.@kwdef struct SimulationParameters
    x0::Vector{Float64}
    n_inputs::Integer
    dt::Float64 = 0.01
    model_dt::Float64 = 0.5
    model_scale::Float64 = 1.0
end
Base.show(io::IO, p::SimulationParameters) = print(io,
    "Simulation Parameters: 
    Initial state: $(p.x0)
    Simulation dt: $(p.dt) 
    Time between model calls: $(p.model_dt)
    Model scale: $(p.model_scale)"
)

Base.@kwdef struct EvaluationParameters
    n_task_executions::Integer = 1
end

struct Spline
    ts::Vector{Float64}
    coeffs_x::Vector{Float64}
    coeffs_y::Vector{Float64}
    x0::Float64
    y0::Float64
end

Base.@kwdef struct RolloutData
    ts::Vector{Float64}
    xs::Matrix{Float64}
    us::Matrix{Float64}
    idx_segs::Vector{<:Integer}     # overall index at each model call
    t0_segs::Vector{Float64}        # time at each model call 
    x0_segs::Matrix{Float64}        # state at each model call
    setpoints::Matrix{Float64}      # Corrected setpoints, from calling model
    xf::Vector{Float64}             # final state, at T+1
    loss::Float64
end

Base.@kwdef struct EvaluationData
    r::RolloutData
    r_no_model::RolloutData         # rollout data w/o using the model       
    xs_task::Vector{Float64}        # x-coords of task
    ys_task::Vector{Float64}        # y-coords of task
end
