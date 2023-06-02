function policy_update!(
    algo::WalkingWindowAlgorithm,
    task::AbstractTask, 
    model::Chain,
    optimizer,
    simple_dynamics::Dynamics,
    actual_dynamics::Dynamics, 
    controller::Controller,
    cost::Cost,
    training_params::TrainingParameters,
    sim_params::SimulationParameters
)
    t0 = 0.0
    x0 = sim_params.x0
    r = rollout_actual_dynamics(
        task, model, actual_dynamics, controller, cost, algo, sim_params, t0, x0, algo.segs_per_rollout
    )
    loss, grads = gradient_estimate(
        r, task, model, simple_dynamics, controller, cost, algo, training_params, sim_params
    )
    update!(optimizer,model,grads[1])
    return loss, [r]
end

function policy_update!(
    algo::RandomInitialAlgorithm,
    task::AbstractTask, 
    model::Chain,
    optimizer,
    simple_dynamics::Dynamics,
    actual_dynamics::Dynamics, 
    controller::Controller,
    cost::Cost,
    training_params::TrainingParameters,
    sim_params::SimulationParameters
)

    rs = Vector{RolloutData}(undef,algo.n_rollouts_per_update)
    for i in 1:algo.n_rollouts_per_update
        if algo.perc_of_task_to_sample == 0
            t0 = 0.0
        else
            t0 = rand(Uniform(0.0, end_time(task)*algo.perc_of_task_to_sample))
        end
        x0 = algo.to_state(evaluate(task,t0)) + rand(MvNormal(diagm(algo.variances)))

        rs[i] = rollout_actual_dynamics(
            task, model, actual_dynamics, controller, cost, algo, sim_params, t0, x0, algo.segs_per_rollout
        )
    end

    loss, grads = gradient_estimate(
        rs, task, model, simple_dynamics, controller, cost, algo, training_params, sim_params
    )
    update!(optimizer,model,grads[1])
    return loss, rs
end

function policy_update!(
    algo::HardwareTrainingAlgorithm,
    connections::Connections,
    task::AbstractTask, 
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