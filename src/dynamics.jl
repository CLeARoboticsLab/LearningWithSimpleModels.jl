abstract type SystemParameters end

abstract type Dyanmics end

Base.@kwdef struct SimpleDynamics <: Dyanmics
    params::SystemParameters
    f::Function
end

Base.@kwdef struct ActualDynamics <: Dyanmics
    params::SystemParameters
    f::Function
end

f_simple(dyn::SimpleDynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)
f_actual(dyn::ActualDynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)

function rollout_actual_dynamics()

end