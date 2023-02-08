function train(;
    simple_dynamics::Dynamics, #TODO: can probably combine these two types
    actual_dynamics::Dynamics,
    controller::Controller,
    cost::Cost,
    task::Spline,
    params::TrainingParameters  
)
    p = ProgressMeter.Progress(params.iters)
    model = make_model(length(params.x0), params.hidden_layer_sizes)
    optimizer = setup(Adam(params.learning_rate), model)

    for i in 1:params.iters
        policy_update!(task, model, simple_dynamics, actual_dynamics, controller, cost, params)
        ProgressMeter.next!(p)
    end

    #TODO this is temp code
    # x0_segs, xs_actual, us_actual = rollout_actual_dynamics(task, model, actual_dynamics, controller, params)
    # display(x0_segs)
end

function policy_update!(
    task::Spline, 
    model::Chain,
    simple_dynamics::Dynamics,
    actual_dynamics::Dynamics, 
    controller::Controller,
    cost::Cost,
    params::TrainingParameters
)
    t0_segs, x0_segs, xs_actual, us_actual, xf = rollout_actual_dynamics(
        task, model, actual_dynamics, controller, params
    )
    task_time, n_segments, segment_length = properties(task, params)

    window_start_idx = 1
    window_end_idx = window_start_idx + params.segs_in_window

    loss = 0.0
    while window_end_idx <= n_segments
        window = window_start_idx:window_end_idx-1

        for j in window
            # Here we are repeating the process found in rollout_actual_dynamics
            # so that we can take derivatives through the simple_dynamics
            x = x0_segs[:,j]
            u = 0.0
            tf_seg = t0_segs[j] + params.model_dt - params.dt
            setpoint = evaluate(task, tf_seg)
            new_setpoint = new_setpoint_from_model(setpoint, model, t0_segs[j], x0_segs[:,j], task_time)

            ts = t0_segs[j]:params.dt:tf_seg
            for (i,t) in enumerate(ts)
                overall_idx = (j-1)*segment_length + i
                u = next_command(controller, x, new_setpoint)
                x_actual_next = overall_idx+1 < size(xs_actual,2) ? xs_actual[:,overall_idx+1] : xf
                x = (
                    f_simple(simple_dynamics, t, params.dt, x, u)
                    - f_simple(simple_dynamics, t, params.dt, xs_actual[:,overall_idx], us_actual[:,overall_idx])
                    + x_actual_next
                )
            end
            loss = loss + stage_cost(cost, x, setpoint, u)
        end

        window_start_idx += 1
        window_end_idx += 1
    end
    println(loss)
end