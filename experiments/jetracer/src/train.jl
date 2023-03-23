function train(;
    simple_dynamics::Dynamics,
    controller::Controller,
    ctrl_params::ControllerParameters,
    cost::Cost,
    task::Spline,
    algo::TrainingAlgorithm,
    training_params::TrainingParameters,
    sim_params::SimulationParameters  
)
    p = ProgressMeter.Progress(training_params.iters)
    model = make_model(length(sim_params.x0), training_params.hidden_layer_sizes)

    if training_params.optim == gradient_descent
        optimizer = setup(Descent(training_params.learning_rate), model)
    else
        optimizer = setup(Adam(training_params.learning_rate), model)
    end

    losses = Vector{Float64}(undef,0)
    rollouts = Vector{RolloutData}(undef,0)
    connections = open_connections()
    sleep(0.5)
    for i in 1:training_params.iters
        loss, r = policy_update!(algo, connections, task, model, optimizer, simple_dynamics,
                                 controller, ctrl_params, cost, sim_params)
        ProgressMeter.next!(p, showvalues = [(:loss,loss)])
        push!(losses, loss)
        push!(rollouts, r)
        plot_rollout(r, task, loss, losses)
    end
    close_connections(connections)

    save_model(training_params, model)
    plot_losses(training_params, losses)

    # TODO need to figure out how to animate rollouts of varying lengths
    # (probably need to make arrays of Point2f)
    # animate_training(training_params, rollouts, task)
end

function policy_update!(
    algo::HardwareTrainingAlgorithm,
    connections::Connections,
    task::Spline, 
    model::Chain,
    optimizer,
    simple_dynamics::Dynamics,
    controller::Controller,
    ctrl_params::ControllerParameters,
    cost::Cost,
    sim_params::SimulationParameters
)
    n_segments = Integer(round(algo.seconds_per_rollout / sim_params.model_dt))
    r = rollout_actual_dynamics(connections, task, model, algo, sim_params, 
                                ctrl_params, n_segments)
    loss, grads = gradient_estimate(r,task,model,simple_dynamics,controller,
                                    cost,algo,sim_params)
    update!(optimizer,model,grads[1])

    return loss, r              
end