f_simple(dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)
f_actual(dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)

# Rollout from the x0 given in sim_params, for the entire task specified by
# eval_params, from the beginning
function rollout_actual_dynamics(
    task::AbstractTask, 
    model::Chain,
    actual_dynamics::Dynamics, 
    controller::Controller,
    cost::Cost,
    sim_params::SimulationParameters,
    eval_params::EvaluationParameters,
    ; use_model = true
)  
    task_time, _ = properties(task, sim_params)
    n_segments = eval_params.n_task_executions * Integer(round(task_time/sim_params.model_dt))
    t0 = 0.0
    x0 = sim_params.x0

    return rollout_actual_dynamics(
        task, model, actual_dynamics, controller, cost, sim_params, t0, x0, n_segments
        ; use_model
    )  
end

# Rollout from the specified t0, x0, for only n_segments model calls 
function rollout_actual_dynamics(
    task::AbstractTask, 
    model::Chain,
    actual_dynamics::Dynamics, 
    controller::Controller,
    cost::Cost, 
    sim_params::SimulationParameters,
    t0::Float64,
    x0::Vector{Float64},
    n_segments::Integer
    ; use_model = true
)  
    task_time, segment_length = properties(task, sim_params)
    total_timesteps = n_segments * segment_length
     
    t0_segs = zeros(n_segments)

    n_states = length(sim_params.x0)
    idx_segs = zeros(Int64, n_segments)
    x0_segs = zeros(n_states, n_segments)
    ts_actual = zeros(total_timesteps)
    xs_actual = zeros(n_states, total_timesteps)
    us_actual = zeros(sim_params.n_inputs, total_timesteps)
    setpoints = zeros(4, n_segments)
    gain_adjs = zeros(5, n_segments)
    ctrl_setpoints = zeros(6, total_timesteps)

    overall_idx = 1
    t = t0
    task_t0 = t
    x = x0
    loss = 0.0
    for j in 1:n_segments
        idx_segs[j] = overall_idx

        # start and end time of this segment
        t0_segs[j] = t
        tf_seg = t + sim_params.model_dt - sim_params.dt

        x0_segs[:,j] = x

        # setpoint is task evaluated at next model call time (segment end time)
        setpoint = evaluate(task, tf_seg)
        new_setpoint, gains_adjustment = call_model(sim_params, setpoint, model, t, x, task_time)
        
        # Discard the new_setpoints and gains if not using the model
        if !use_model
            new_setpoint = setpoint
            gains_adjustment = zeros(5)
        end 

        setpoints[:,j] = new_setpoint
        gain_adjs[:,j] = gains_adjustment

        # Generate a spline from the current point to the new setpoint that the
        # low level controller will track
        prev_setpoint = j > 1 ? setpoints[:,j-1] : evaluate(task, t)
        spline_seg = spline_segment(t, tf_seg, prev_setpoint, new_setpoint)

        # rollout on this segment
        for _ in 1:segment_length
            ts_actual[overall_idx] = t
            xs_actual[:,overall_idx] = x
            ctrl_setpoints[:,overall_idx] = evaluate_segment(spline_seg, t+sim_params.dt)
            u = next_command(controller, x, ctrl_setpoints[:,overall_idx], gains_adjustment)
            us_actual[:,overall_idx] = u
            loss = loss + stage_cost(cost, x, evaluate(task, t), u)

            overall_idx += 1
            t += sim_params.dt
            x = f_actual(actual_dynamics, t, sim_params.dt, x, u)
        end
    end
    xf = x
    return RolloutData(;
        ts = ts_actual,
        xs = xs_actual,
        us = us_actual,
        task_t0 = task_t0,
        idx_segs = idx_segs,
        t0_segs = t0_segs,
        x0_segs = x0_segs,
        setpoints = setpoints,
        gain_adjs = gain_adjs,
        ctrl_setpoints = ctrl_setpoints,
        xf = xf,
        loss = loss
    )
end
