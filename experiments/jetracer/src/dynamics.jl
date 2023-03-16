function rollout_actual_dynamics(
    connections::Connections,
    task::Spline,
    model::Chain,
    sim_params::SimulationParameters,
    n_segments::Integer
)

    task_time, _ = properties(task, sim_params)

    for j in 1:n_segments
        do_and_wait(sim_params.model_dt) do 
            
            t = time_elapsed(connections) # TODO figure out adding this to t0
            x = state(connections)
            setpoint = evaluate(task, t + sim_params.model_dt)
            new_setpoint, gains_adjustment = call_model(sim_params, setpoint, model, t, x, task_time)
        end 
    end

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
