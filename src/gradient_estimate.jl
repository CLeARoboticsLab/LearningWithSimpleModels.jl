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
    rs = [r]
    return gradient_estimate(
        rs, task, model, simple_dynamics, controller, cost,
        algo, training_params, sim_params
    )
end

function gradient_estimate(
    rs::Vector{RolloutData},
    task::Spline, 
    model::Chain,
    simple_dynamics::Dynamics,
    controller::Controller,
    cost::Cost,
    algo::TrainingAlgorithm,
    training_params::TrainingParameters,
    sim_params::SimulationParameters
)
    task_time, segment_length = properties(task, sim_params)

    loss, grads = withgradient(model) do model
        loss = 0.0
        for r in rs
            n_segments = length(r.t0_segs)
            window_start_idx = 1
            window_end_idx = window_start_idx + algo.segs_in_window - 1
            while window_end_idx <= n_segments
                window = window_start_idx:window_end_idx
                for j in window
                    # Here we are repeating the process found in rollout_actual_dynamics
                    # so that we can take derivatives through the simple_dynamics
                    seg_start_idx = r.idx_segs[j]
                    t = r.t0_segs[j]
                    x = r.x0_segs[:,j]
                    u = 0.0
                    tf_seg = t + sim_params.model_dt - sim_params.dt
                    setpoint = evaluate(task, tf_seg)
                    new_setpoint = new_setpoint_from_model(sim_params, setpoint, model, t, x, task_time)
        
                    for i in 0:segment_length-1
                        overall_idx = seg_start_idx + i
                        u = next_command(controller, x, new_setpoint)
                        if training_params.loss_aggregation == simulation_timestep
                            loss = loss + stage_cost(cost, x, evaluate(task, t), u)
                        end
                        t = t + sim_params.dt
                        x_actual_next = overall_idx+1 <= size(r.xs,2) ? r.xs[:,overall_idx+1] : r.xf
                        x = (
                            f_simple(simple_dynamics, t, sim_params.dt, x, u)
                            - f_simple(simple_dynamics, t, sim_params.dt, r.xs[:,overall_idx], r.us[:,overall_idx])
                            + x_actual_next
                        )
                        if !isapprox(x, x_actual_next)
                            error("Mismatched states at index $(overall_idx)
                            x: $(x)
                            x_actual_next: $(x_actual_next)")
                        end
                    end
                    if training_params.loss_aggregation == model_call
                        loss = loss + stage_cost(cost, x, setpoint, u)
                    end
                end
                window_start_idx += 1
                window_end_idx += 1
            end
        end
        return loss
    end
    return loss, grads
end
