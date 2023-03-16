Base.@kwdef struct JetsonControllerParameters <: ControllerParameters
    kx::Float64
    ky::Float64
    kv::Float64
    kϕ::Float64
    limit::Bool
    a_limit::Float64
    ω_limit::Float64
end

jetracer_controller_parameters() = JetsonControllerParameters(;
    kx = 0.75,
    ky = 0.75,
    kv = 0.75,
    kϕ = 0.75,
    limit = true,
    a_limit = 1.0,
    ω_limit = 1.0
)