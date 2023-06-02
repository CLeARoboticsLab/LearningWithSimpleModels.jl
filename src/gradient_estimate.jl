function gradient_estimate(
    r::RolloutData,
    task::AbstractTask, 
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
    task::AbstractTask, 
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
            window_start_idx = 1 + algo.n_beginning_segs_to_truncate 
            window_end_idx = window_start_idx + algo.segs_in_window - 1
            while window_end_idx <= n_segments
                window = window_start_idx:window_end_idx
                if window_start_idx > 1
                    prev_setpoint = r.setpoints[:,window_start_idx-1]
                else
                    prev_setpoint = evaluate(task, r.t0_segs[window_start_idx])
                end
                for j in window
                    # Here we are repeating the process found in rollout_actual_dynamics
                    # so that we can take derivatives through the simple_dynamics
                    seg_start_idx = r.idx_segs[j]
                    t = r.t0_segs[j]
                    x = r.x0_segs[:,j]
                    u = 0.0
                    tf_seg = t + sim_params.model_dt - sim_params.dt
                    setpoint = evaluate(task, tf_seg)
                    t_in = t
                    if !isnothing(algo.task_time_est)
                        t_in = algo.task_time_est(task, t, x)
                    end
                    new_setpoint, gains_adjustment = call_model(sim_params, setpoint, model, t_in, x, task_time)
                    spline_seg = spline_segment(sim_params.spline_seg_type ,t, tf_seg, prev_setpoint, new_setpoint)
                    prev_setpoint = new_setpoint

                    for i in 0:segment_length-1
                        overall_idx = seg_start_idx + i
                        ctrl_setpoint = evaluate_segment(spline_seg, t+sim_params.dt)
                        u = next_command(controller, x, ctrl_setpoint, gains_adjustment)
                        if training_params.loss_aggregation == simulation_timestep
                            loss = loss + stage_cost(cost, t, x, evaluate(task, t), task, u, simple_dynamics)
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
                        loss = loss + stage_cost(cost, t, x, setpoint, task, u, simple_dynamics)
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

function gradient_estimate(
    r::RolloutData,
    task::AbstractTask,
    model::Chain,
    simple_dynamics::Dynamics,
    controller::Controller,
    cost::Cost,
    algo::HardwareTrainingAlgorithm,
    sim_params::SimulationParameters
)
    task_time = end_time(task)

    loss, grads = withgradient(model) do model
        loss = 0.0
        n_segments = length(r.t0_segs)
        window_start_idx = 1 + algo.n_beginning_segs_to_truncate
        window_end_idx = algo.use_window ? window_start_idx + algo.segs_in_window - 1 : n_segments-2
        while window_end_idx <= n_segments-2 # TODO: I think there is a race condition on the cpp side forcing -2 here
            window = window_start_idx:window_end_idx
            prev_setpoint = window_start_idx==1 ? starting_setpoint(r.x0_segs[:,1]) : r.setpoints[:,window_start_idx-1]
            t = 0.0
            x = 0.0
            u = 0.0
            for j in window
                t0_seg = r.t0_segs[j]
                tf_seg = t0_seg + sim_params.model_dt
                x = r.x0_segs[:,j]
                
                setpoint = evaluate(task, tf_seg)
                t_in = t0_seg
                if !isnothing(algo.task_time_est)
                    t_in = algo.task_time_est(task, t0_seg, x)
                end
                new_setpoint, gains_adjustment = call_model(sim_params, setpoint, 
                                                            model, t_in, x, task_time)
                t = t0_seg - r.task_t0
                spline_seg = spline_segment(sim_params.spline_seg_type, t, t + sim_params.model_dt, prev_setpoint, new_setpoint)
                prev_setpoint = new_setpoint

                idx_seg = r.idx_segs[j]
                x = r.xs[:,idx_seg]
                for i in r.idx_segs[j]:r.idx_segs[j+1]-1
                    t = r.ts[i]
                    ctrl_setpoint = evaluate_segment(spline_seg, t)
                    u = next_command(controller, x, ctrl_setpoint, gains_adjustment)
                    loss = loss + stage_cost(cost, r.task_t0 + t, x, evaluate(task, r.task_t0 + t), task, u, simple_dynamics)
                    x_actual_next = r.xs[:,i+1]
                    x = (
                        f_simple(simple_dynamics, t, sim_params.dt, x, u)
                        - f_simple(simple_dynamics, t, sim_params.dt, r.xs[:,i], r.us[:,i])
                        + x_actual_next
                    )
                    if !isapprox(x, x_actual_next)
                        println("Mismatched x at index $(i), t: $(t)
                        udiff: $(u - r.us[:,i])
                        u: $(u)
                        r.u: $(r.us[:,i])
                        xdiff: $(x - x_actual_next)")
                    end
                end
            end
            window_start_idx += 1
            window_end_idx += 1
        end
        return loss
    end
    return loss, grads
end