struct Spline
    ts::Vector{Float64}
    coeffs_x::Vector{Float64}
    coeffs_y::Vector{Float64}
    x0::Float64
    y0::Float64
end

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

Base.@kwdef struct TrainingParameters
    hidden_layer_sizes::Vector{<:Integer} = [64, 64]
    learning_rate::Float64 = 1e-3
    iters::Integer = 50
    segs_in_window::Integer = 5
    save_path = nothing
end

Base.@kwdef struct SimulationParameters
    x0::Vector{Float64}
    n_inputs::Integer
    dt::Float64 = 0.01
    model_dt::Float64 = 0.5
end