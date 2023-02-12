f_simple(dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)
f_actual(dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)

# Rollout from the x0 given in sim_params, for the entire task from the
# beginning, including task repeats
function rollout_actual_dynamics(
    task::Spline, 
    model::Chain,
    actual_dynamics::Dynamics, 
    controller::Controller, 
    sim_params::SimulationParameters
    ; use_model = true
)  
    _, n_segments, _, _ = properties(task, sim_params)
    t0 = 0.0
    x0 = sim_params.x0

    return rollout_actual_dynamics(
        task, model, actual_dynamics, controller, sim_params, t0, x0, n_segments
        ; use_model
    )  
end

# Rollout from the specified t0, x0, for only n_segments model calls 
function rollout_actual_dynamics(
    task::Spline, 
    model::Chain,
    actual_dynamics::Dynamics, 
    controller::Controller, 
    sim_params::SimulationParameters,
    t0::Float64,
    x0::Vector{Float64},
    n_segments::Integer
    ; use_model = true
)  
    task_time, _, segment_length, _ = properties(task, sim_params)
    total_timesteps = n_segments * segment_length
     
    t0_segs = zeros(n_segments)

    n_states = length(sim_params.x0)
    x0_segs = zeros(n_states, n_segments)
    ts_actual = zeros(total_timesteps)
    xs_actual = zeros(n_states, total_timesteps)
    us_actual = zeros(sim_params.n_inputs, total_timesteps)

    x = x0
    for j in 1:n_segments
        # start and end time of this segment
        t0_seg = t0 + (j-1)*sim_params.model_dt
        t0_segs[j] = t0_seg
        tf_seg = t0_seg + sim_params.model_dt - sim_params.dt

        x0_segs[:,j] = x

        # setpoint is task evaluated at next model call time (segment end time)
        setpoint = evaluate(task, tf_seg)
        new_setpoint = new_setpoint_from_model(sim_params, setpoint, model, t0_seg, x, task_time)
        
        # Discard the new_setpoint if not using the model
        if !use_model
            new_setpoint = setpoint
        end 

        # rollout on this segment
        ts = t0_seg:sim_params.dt:tf_seg
        for (i,t) in enumerate(ts)
            overall_idx = (j-1)*segment_length + i
            ts_actual[overall_idx] = overall_idx * sim_params.dt
            xs_actual[:,overall_idx] = x
            u = next_command(controller, x, new_setpoint)
            us_actual[:,overall_idx] = u
            x = f_actual(actual_dynamics, t, sim_params.dt, x, u)
        end
    end
    xf = x
    return RolloutData(;
        ts = ts_actual,
        xs = xs_actual,
        us = us_actual,
        t0_segs = t0_segs,
        x0_segs = x0_segs,
        xf = xf
    )
end
