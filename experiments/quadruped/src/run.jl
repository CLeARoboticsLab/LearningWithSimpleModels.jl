function run()
    task = quadruped_figure_eight_task()
    algo = quadruped_training_algorithm()
    training_params = quadruped_training_parameters()
    sim_params = quadruped_simulation_parameters()
    ctrl_params = quadruped_controller_parameters()
    simple_dynamics = quadruped_simple_dynamics()
    controller = quadruped_controller()
    cost = quadruped_cost()

    train(
        simple_dynamics;
        controller = controller,
        ctrl_params = ctrl_params,
        cost = cost,
        task = task,
        algo = algo,
        training_params = training_params,
        sim_params = sim_params
    )
end

function evaluate_no_model()
    evaluate_on_hardware(;
        task = quadruped_figure_eight_task(),
        algo = quadruped_training_algorithm(),
        ctrl_params = quadruped_controller_parameters(),
        sim_params = quadruped_simulation_parameters(),
        eval_params = quadruped_evaluation_parameters(),
        use_model = false
    )
end

function evaluate_model()
    evaluate_on_hardware(;
        task = quadruped_figure_eight_task(),
        algo = quadruped_training_algorithm(),
        ctrl_params = quadruped_controller_parameters(),
        sim_params = quadruped_simulation_parameters(),
        eval_params = quadruped_evaluation_parameters(),
        use_model = true
    )
end

function make_eval_plots()
    plot_hardware_evaluation(;
        eval_params = quadruped_evaluation_parameters(),
        task = quadruped_figure_eight_task(),
        algo = quadruped_training_algorithm(),
        training_params = quadruped_training_parameters(),
        sim_params = quadruped_simulation_parameters()
    )
end