function run_open_loop()
    task = dp_task_open_loop()
    algo = dp_training_algorithm_open_loop()
    training_params = dp_training_parameters_open_loop()
    sim_params = dp_simulation_parameters_open_loop()
    simple_dynamics = dp_simple_dynamics_open_loop()
    actual_dynamics = dp_actual_dynamics_open_loop()
    controller = dp_controller_open_loop()
    cost = dp_cost_open_loop()
    eval_params = dp_evaluation_parameters_open_loop()

    train(
        simple_dynamics,
        actual_dynamics;
        controller = controller,
        cost = cost,
        task = task,
        algo = algo,
        training_params = training_params,
        sim_params = sim_params
    )
    evaluate_model(;
        actual_dynamics = actual_dynamics,
        controller = controller,
        cost = cost,
        task = task,
        algo = algo,
        training_params = training_params,
        sim_params = sim_params,
        eval_params = eval_params,
        model = nothing
    )
end

function run_open_loop_evaluation()
    task = dp_task_open_loop()
    algo = dp_training_algorithm_open_loop()
    training_params = dp_training_parameters_open_loop()
    sim_params = dp_simulation_parameters_open_loop()
    simple_dynamics = dp_simple_dynamics_open_loop()
    actual_dynamics = dp_actual_dynamics_open_loop()
    controller = dp_controller_open_loop()
    cost = dp_cost_open_loop()
    eval_params = dp_evaluation_parameters_open_loop()

    evaluate_model(;
        actual_dynamics = actual_dynamics,
        controller = controller,
        cost = cost,
        task = task,
        algo = algo,
        training_params = training_params,
        sim_params = sim_params,
        eval_params = eval_params,
        model = nothing
    )
end