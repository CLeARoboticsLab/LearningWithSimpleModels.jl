function train(;
    simple_dynamics::SimpleDynamics,
    actual_dynamics::ActualDynamics,
    controller::Controller,
    cost::Cost,
    task::Spline,
    params::TrainingParameters  
)
    p = ProgressMeter.Progress(params.iters)
    model = make_model(params.n_states, params.hidden_layer_sizes)
    optimizer = setup(Adam(params.learning_rate), model)

    for i in 1:params.iters
        policy_update!()
        ProgressMeter.next!(p)
    end

    #TODO this is temp code
    rollout_actual_dynamics(task, params)
end

function policy_update!()
    
end