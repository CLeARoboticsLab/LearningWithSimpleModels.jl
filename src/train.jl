function train(;
    simple_dynamics::SimpleDynamics, #TODO: can probably combine these two types
    actual_dynamics::ActualDynamics,
    controller::Controller,
    cost::Cost,
    task::Spline,
    params::TrainingParameters  
)
    p = ProgressMeter.Progress(params.iters)
    model = make_model(length(params.x0), params.hidden_layer_sizes)
    optimizer = setup(Adam(params.learning_rate), model)

    for i in 1:params.iters
        policy_update!(task, model, actual_dynamics, controller, params)
        ProgressMeter.next!(p)
    end

    #TODO this is temp code
    # x0_segs, xs_actual, us_actual = rollout_actual_dynamics(task, model, actual_dynamics, controller, params)
    # display(x0_segs)
end

function policy_update!(
    task::Spline, 
    model::Chain,
    actual_dynamics::Dynamics, 
    controller::Controller, 
    params::TrainingParameters
)
    x0_segs, xs_actual, us_actual = rollout_actual_dynamics(task, model, actual_dynamics, controller, params)
    n_segments = size(x0_segs,2)

    window_start_idx = 1
    window_end_idx = window_start_idx + params.segs_in_window

    while window_end_idx <= n_segments
        window = window_start_idx:window_end_idx-1

        # for i in window

        # end

        window_start_idx += 1
        window_end_idx += 1
    end

end