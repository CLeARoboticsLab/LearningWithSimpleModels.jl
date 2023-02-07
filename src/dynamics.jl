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
    
    x = params.x0
    for j in 1:n_segments
        # start and end time of this segment
        t0_seg = (j-1)*params.model_dt
        tf_seg = t0_seg + params.model_dt - params.dt
        
        # setpoint is task evaluated at next model call time (segment end time)
        setpoint = evaluate(task, tf_seg)
        new_setpoint = new_setpoint_from_model(setpoint, model, t0_seg, x, task_time)

        ts = t0_seg:params.dt:tf_seg
        for (i,t) in enumerate(ts)
            u = next_command(controller, x, new_setpoint)
            x = f_actual(dynamics, t, params.dt, x, u)
        end
    end
end
