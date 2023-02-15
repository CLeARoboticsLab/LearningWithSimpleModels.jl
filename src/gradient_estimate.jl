function gradient_estimate(
    r::RolloutData,
    task::Spline, 
    model::Chain,
    simple_dynamics::Dynamics,
    controller::Controller,
    cost::Cost,
    algo::TrainingAlgorithm,
    training_params::TrainingParameters,
    sim_params::SimulationParameters
)
    task_time, _, segment_length, _ = properties(task, sim_params)
    n_segments = length(r.t0_segs)

    window_start_idx = 1
    window_end_idx = window_start_idx + training_params.segs_in_window - 1
    
    loss, grads = withgradient(model) do model
        loss = 0.0
        while window_end_idx <= n_segments
            window = window_start_idx:window_end_idx
            for j in window
                # Here we are repeating the process found in rollout_actual_dynamics
                # so that we can take derivatives through the simple_dynamics
                x = r.x0_segs[:,j]
                u = 0.0
                tf_seg = r.t0_segs[j] + sim_params.model_dt - sim_params.dt
                setpoint = evaluate(task, tf_seg)
                new_setpoint = new_setpoint_from_model(sim_params, setpoint, model, r.t0_segs[j], r.x0_segs[:,j], task_time)
    
                ts = r.t0_segs[j]:sim_params.dt:tf_seg
                for (i,t) in enumerate(ts)
                    overall_idx = (j-1)*segment_length + i
                    u = next_command(controller, x, new_setpoint)
                    if typeof(training_params.loss_aggregation) == AtSimulationTimestep
                        loss = loss + stage_cost(cost, x, evaluate(task, t), u)
                    end
                    x_actual_next = overall_idx+1 < size(r.xs,2) ? r.xs[:,overall_idx+1] : r.xf
                    x = (
                        f_simple(simple_dynamics, t, sim_params.dt, x, u)
                        - f_simple(simple_dynamics, t, sim_params.dt, r.xs[:,overall_idx], r.us[:,overall_idx])
                        + x_actual_next
                    )
                end
                if typeof(training_params.loss_aggregation) == AtModelCall
                    loss = loss + stage_cost(cost, x, setpoint, u)
                end
            end
            window_start_idx += 1
            window_end_idx += 1
        end
        return loss
    end
    return loss, grads
end
