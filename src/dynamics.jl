abstract type SystemParameters end

abstract type Dyanmics end

Base.@kwdef struct SimpleDynamics <: Dyanmics
    params::SystemParameters
    f::Function
    dt::Float64
end

Base.@kwdef struct ActualDynamics <: Dyanmics
    params::SystemParameters
    f::Function
    dt::Float64
end

f_simple(dyn::SimpleDynamics, t, x, u) = dyn.f(dyn,t,x,u)
