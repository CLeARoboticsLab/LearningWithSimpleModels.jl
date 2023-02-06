abstract type SystemParameters end

abstract type Dyanmics end

Base.@kwdef struct SimpleDynamics <: Dyanmics
    params::SystemParameters
    dt::Float64 #TODO: make dt an experiemtnt param?
    f::Function
end

Base.@kwdef struct ActualDynamics <: Dyanmics
    params::SystemParameters
    dt::Float64
    f::Function
end

f_simple(dyn::SimpleDynamics, t::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,x,u)
f_actual(dyn::ActualDynamics, t::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,x,u)