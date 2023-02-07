f_simple(dyn::SimpleDynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)
f_actual(dyn::ActualDynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)

function rollout_actual_dynamics(task::Spline, params::TrainingParameters)
    segment_length = Integer(round(params.model_dt / params.dt))
    n_segments = Integer(round(end_time(task)/params.model_dt))
end
