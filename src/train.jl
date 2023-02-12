function train(;
    simple_dynamics::Dynamics,
    actual_dynamics::Dynamics,
    controller::Controller,
    cost::Cost,
    task::Spline,
    training_params::TrainingParameters,
    sim_params::SimulationParameters  
)
    p = ProgressMeter.Progress(training_params.iters)
    model = make_model(length(sim_params.x0), training_params.hidden_layer_sizes)
    optimizer = setup(Adam(training_params.learning_rate), model)

    losses = zeros(training_params.iters)
    for i in 1:training_params.iters
        loss = policy_update!(
            task, model, optimizer, simple_dynamics, actual_dynamics,
            controller, cost, training_params, sim_params
        )
        ProgressMeter.next!(p, showvalues = [(:loss,loss)])
        losses[i] = loss
    end

    if !isnothing(training_params.save_path)
        @save training_params.save_path model
    end
    
    plot_losses(training_params, losses)

    return model, losses
end

function policy_update!(
    task::Spline, 
    model::Chain,
    optimizer,
    simple_dynamics::Dynamics,
    actual_dynamics::Dynamics, 
    controller::Controller,
    cost::Cost,
    training_params::TrainingParameters,
    sim_params::SimulationParameters
)
    r = rollout_actual_dynamics(task, model, actual_dynamics, controller, sim_params)
    loss, grads = gradient_estimate(
        r, task, model, simple_dynamics, controller, cost, training_params, sim_params
    )
    update!(optimizer,model,grads[1])
    return loss
end

function gradient_estimate(
    r::RolloutData,
    task::Spline, 
    model::Chain,
    simple_dynamics::Dynamics,
    controller::Controller,
    cost::Cost,
    training_params::TrainingParameters,
    sim_params::SimulationParameters
)
    task_time, _, segment_length, _ = properties(task, sim_params)
    n_segments = length(r.t0_segs)

    window_start_idx = 1
    window_end_idx = window_start_idx + training_params.segs_in_window
    
    loss, grads = withgradient(model) do model
        loss = 0.0
        while window_end_idx <= n_segments
            window = window_start_idx:window_end_idx-1
    
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
                    x_actual_next = overall_idx+1 < size(r.xs,2) ? r.xs[:,overall_idx+1] : r.xf
                    x = (
                        f_simple(simple_dynamics, t, sim_params.dt, x, u)
                        - f_simple(simple_dynamics, t, sim_params.dt, r.xs[:,overall_idx], r.us[:,overall_idx])
                        + x_actual_next
                    )
                end
                loss = loss + stage_cost(cost, x, setpoint, u) #TODO: compute this at every dt instead?
            end
            window_start_idx += 1
            window_end_idx += 1
        end
        return loss
    end
    return loss, grads
end
