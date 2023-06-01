function run()
    task = quadruped_figure_eight_task()
    algo = quadruped_training_algorithm()
    training_params = quadruped_training_parameters()
    sim_params = quadruped_simulation_parameters()
    ctrl_params = quadruped_controller_parameters()
    simple_dynamics = quadruped_simple_dynamics()
    actual_dynamics = quadruped_actual_dynamics()
    controller = quadruped_controller()
    cost = quadruped_cost()
    eval_params = quadruped_evaluation_parameters()

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