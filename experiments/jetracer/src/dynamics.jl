function rollout_actual_dynamics(
    connections::Connections,
    task::Spline,
    model::Chain,
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
    gain_adjs = zeros(4, n_segments)
    
    x = state(connections)
    task_t0 = estimate_task_t0(task, x)

    start_rollout(connections)
    for j in 1:n_segments
        do_and_wait(sim_params.model_dt) do 
            t = time_elapsed(connections)
            t0_seg = task_t0 + t
            tf_seg = t0_seg + sim_params.model_dt
            t0_segs[j] = t0_seg

            x = state(connections)
            x0_segs[:,j] = x

            setpoint = evaluate(task, tf_seg)
            new_setpoint, gains_adjustment = call_model(sim_params, setpoint, 
                                                        model, t0_seg, x, task_time)
            
            # Discard the new_setpoints and gains if not using the model
            if !use_model
                new_setpoint = setpoint
                gains_adjustment = zeros(4)
            end 
            
            setpoints[:,j] = new_setpoint
            gain_adjs[:,j] = gains_adjustment
            prev_setpoint = j > 1 ? setpoints[:,j-1] : starting_setpoint(x)
            spline_seg = spline_segment(t, t + sim_params.model_dt, prev_setpoint, new_setpoint)
            send_command(connections, ctrl_params, spline_seg, gains_adjustment)
        end 
    end
    stop_rollout(connections)
    sleep(3.0)
    
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
        ctrl_setpoints = zeros(4, length(rdata.ts)), # TODO
        xf = zeros(4), # TODO; this is probably not needed
        loss = 0.0# TODO; this might not be needed
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
function estimate_task_t0(task::Spline, x::Vector{Float64}, n_points::Integer = 100, ϕ_weight = 0.25)
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
function starting_setpoint(x::Vector{Float64}; v_init = 0.1)
    return [
        x[1],
        x[2],
        max(v_init, x[3])*cos(x[4]),
        max(v_init, x[3])*sin(x[4])
    ]
end