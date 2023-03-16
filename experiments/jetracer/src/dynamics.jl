function rollout_actual_dynamics(
    connections::Connections,
    task::Spline,
    model::Chain,
    sim_params::SimulationParameters,
    ctrl_params::ControllerParameters,
    n_segments::Integer
)

    task_time, _ = properties(task, sim_params)

    setpoints = zeros(4, n_segments)
    
    task_t0 = 0.0 # TODO: calc this from position on track

    start_rollout(connections)
    for j in 1:n_segments
        do_and_wait(sim_params.model_dt) do 
            
            t = time_elapsed(connections) # TODO figure out adding this to t0
            t0_seg = task_t0 + t
            tf_seg = t0_seg + sim_params.model_dt

            x = state(connections)
            setpoint = evaluate(task, tf_seg)
            new_setpoint, gains_adjustment = call_model(sim_params, setpoint, model, t0_seg, x, task_time)
            setpoints[:,j] = new_setpoint
            prev_setpoint = j > 1 ? setpoints[:,j-1] : evaluate(task, t0_seg)
            spline_seg = spline_segment(t, t + sim_params.model_dt, prev_setpoint, new_setpoint)
            send_command(connections, ctrl_params, spline_seg, gains_adjustment)
        end 
    end
    stop_rollout(connections)
end

# Executes the function f and waits for the specified delay. The timer starts
# before f is called
function do_and_wait(f, delay)
    cond = Condition()
    Timer(x->notify(cond), delay)
    data = @async $f()
    wait(cond)
    return fetch(data)
end
