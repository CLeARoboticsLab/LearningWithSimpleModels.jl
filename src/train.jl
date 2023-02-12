function train(;
    simple_dynamics::Dynamics,
    actual_dynamics::Dynamics,
    controller::Controller,
    cost::Cost,
    task::Spline,
    algo::TrainingAlgorithm,
    training_params::TrainingParameters,
    sim_params::SimulationParameters  
)
    p = ProgressMeter.Progress(training_params.iters)
    model = make_model(length(sim_params.x0), training_params.hidden_layer_sizes)
    optimizer = setup(Adam(training_params.learning_rate), model)

    losses = zeros(training_params.iters)
    for i in 1:training_params.iters
        loss = policy_update!(
            algo, task, model, optimizer, simple_dynamics, actual_dynamics,
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
    algo::WalkingWindowAlgorithm,
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
    r = rollout_actual_dynamics(task, model, actual_dynamics, controller, cost, sim_params)
    loss, grads = gradient_estimate(
        r, task, model, simple_dynamics, controller, cost, algo, training_params, sim_params
    )
    update!(optimizer,model,grads[1])
    return loss
end

function policy_update!(
    algo::RandomInitialAlgorithm,
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
    t0 = rand(Uniform(0.0, end_time(task)))
    x0 = algo.to_state(evaluate(task,t0)) + rand(MvNormal(diagm(algo.variances)))

    r = rollout_actual_dynamics(
        task, model, actual_dynamics, controller, cost, sim_params, t0, x0, training_params.segs_in_window
    )

    loss, grads = gradient_estimate(
        r, task, model, simple_dynamics, controller, cost, algo, training_params, sim_params
    )
    update!(optimizer,model,grads[1])

    return loss
end