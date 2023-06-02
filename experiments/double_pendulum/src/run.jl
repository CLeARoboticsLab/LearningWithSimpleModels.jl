function run()
    task = dp_task()
    algo = dp_training_algorithm()
    training_params = dp_training_parameters()
    sim_params = dp_simulation_parameters()
    ctrl_params = dp_controller_parameters()
    simple_dynamics = dp_simple_dynamics()
    actual_dynamics = dp_actual_dynamics()
    controller = dp_controller()
    cost = dp_cost()
    eval_params = dp_evaluation_parameters()

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