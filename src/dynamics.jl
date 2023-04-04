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
    algo::TrainingAlgorithm,
    sim_params::SimulationParameters,
    eval_params::EvaluationParameters,
    ; use_model = true
)  
    task_time, _ = properties(task, sim_params)
    n_segments = eval_params.n_task_executions * Integer(round(task_time/sim_params.model_dt))
    t0 = 0.0
    x0 = sim_params.x0

    return rollout_actual_dynamics(
        task, model, actual_dynamics, controller, cost, algo, sim_params, t0, x0, n_segments
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
    algo::TrainingAlgorithm,
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
    gain_adjs = zeros(6, n_segments)
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
        t_in = t
        if !isnothing(algo.task_time_est)
            t_in = algo.task_time_est(task, t, x)
        end
        new_setpoint, gains_adjustment = call_model(sim_params, setpoint, model, t_in, x, task_time)
        
        # Discard the new_setpoints and gains if not using the model
        if !use_model
            new_setpoint = setpoint
            gains_adjustment = zeros(6)
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
            loss = loss + stage_cost(cost, t, x, evaluate(task, t), task, u)

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

# Rollout on hardware, for n_segments model calls 
function rollout_actual_dynamics(
    connections::Connections,
    task::AbstractTask,
    model::Chain,
    algo::HardwareTrainingAlgorithm,
    sim_params::SimulationParameters,
    ctrl_params::ControllerParameters,
    n_segments::Integer
    ; use_model = true
)
    task_time = end_time(task)
    n_states = length(sim_params.x0)

    t0_segs = zeros(n_segments)
    x0_segs = zeros(n_states, n_segments)
    setpoints = zeros(4, n_segments)
    gain_adjs = zeros(6, n_segments)
    
    x = state(connections)
    task_t0 = estimate_task_t0(task, x)

    start_rollout(connections)
    for j in 1:n_segments + algo.stopping_segments
        do_and_wait(sim_params.model_dt) do 
            if j <= n_segments
                t = time_elapsed(connections)
            else
                t = t0_segs[n_segments] - task_t0 + (j-n_segments)*sim_params.model_dt
            end
            t0_seg = task_t0 + t
            tf_seg = t0_seg + sim_params.model_dt
            x = state(connections)
            setpoint = evaluate(task, tf_seg)
            new_setpoint, gains_adjustment = call_model(sim_params, setpoint, 
                                                        model, t0_seg, x, task_time)
            
            # Discard the new_setpoints and gains if not using the model
            if !use_model || j > n_segments
                new_setpoint = setpoint
                gains_adjustment = zeros(6)
            end 
            
            prev_setpoint = j in 2:n_segments ? setpoints[:,j-1] : starting_setpoint(x)
            spline_seg = spline_segment(t, t + sim_params.model_dt, prev_setpoint, new_setpoint)
            send_command(connections, ctrl_params, spline_seg, gains_adjustment)

            if j == n_segments
                stop_rollout(connections)
            end

            if j in 1:n_segments
                t0_segs[j] = t0_seg
                x0_segs[:,j] = x
                setpoints[:,j] = new_setpoint
                gain_adjs[:,j] = gains_adjustment
            end
        end 
    end
    
    sleep(5.5)
    
    rdata = rollout_data(connections)
    return RolloutData(;
        ts = rdata.ts,
        xs = rdata.xs,
        us = rdata.us,
        task_t0 = task_t0,
        idx_segs = rdata.seg_idxs,
        t0_segs = t0_segs,
        x0_segs = x0_segs,
        setpoints = setpoints,
        gain_adjs = gain_adjs,
        ctrl_setpoints = rdata.ctrl_setpoints,
        xf = zeros(4), # not used
        loss = 0.0 # not used
    )
end

# Executes the function f and waits for the specified delay. The timer starts
# before f is called
function do_and_wait(f::Function, delay::Real)
    cond = Condition()
    Timer(x->notify(cond), delay)
    data = @async $f()
    wait(cond)
    return fetch(data)
end

# Estimates progess along a task based on the current state. Discretizes the
# task into n_points, calculates a cost for each point (distance to each point +
# ϕ_weight * different in heading angle), and outputs the task time which
# minimizes this cost
function estimate_task_t0(task::AbstractTask, x::Vector{Float64}, n_points::Integer = 100, ϕ_weight = 0.25)
    dt = end_time(task) / n_points
    costs = zeros(n_points)
    for i in 1:n_points
        p = evaluate(task, dt*i)
        dist = norm(x[1:2] - p[1:2])
        Δϕ = abs(x[4] - atan(p[4], p[3]))
        costs[i] = dist + ϕ_weight*Δϕ
    end
    return argmin(costs) * dt
end

# When generating the very first spline segment, use the current state of the
# model instead of a point on the task to ensure the agent starts smoothly. Use
# a small minimum initial velocity to ensure the spline "points" the right way
# in the beginning
function starting_setpoint(x::Vector{Float64}; v_init = 0.01)
    return [
        x[1],
        x[2],
        max(v_init, x[3])*cos(x[4]),
        max(v_init, x[3])*sin(x[4]),
        0.,
        0.
    ]
end