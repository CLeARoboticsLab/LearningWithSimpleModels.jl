f_simple(dyn::SimpleDynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)
f_actual(dyn::ActualDynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)

function rollout_actual_dynamics(
    task::Spline, 
    model::Chain,
    dynamics::Dynamics, 
    controller::Controller, 
    params::TrainingParameters
)
    task_time = end_time(task)
    n_segments = Integer(round(task_time/params.model_dt))
    
    n_states = length(params.x0)
    x0_segs = zeros(n_states, n_segments)

    xs_actual = zeros(n_states, Integer(round(task_time/params.dt)))
    us_actual = zeros(params.n_inputs, Integer(round(task_time/params.dt)))

    x = params.x0
    for j in 1:n_segments
        # start and end time of this segment
        t0_seg = (j-1)*params.model_dt
        tf_seg = t0_seg + params.model_dt - params.dt
        
        x0_segs[:,j] = x

        # setpoint is task evaluated at next model call time (segment end time)
        setpoint = evaluate(task, tf_seg)
        new_setpoint = new_setpoint_from_model(setpoint, model, t0_seg, x, task_time)

        # rollout on this segment
        ts = t0_seg:params.dt:tf_seg
        for (i,t) in enumerate(ts)
            xs_actual[:,i] = x
            u = next_command(controller, x, new_setpoint)
            us_actual[:,i] = u
            x = f_actual(dynamics, t, params.dt, x, u)
        end
    end
    return x0_segs, xs_actual, us_actual
end
