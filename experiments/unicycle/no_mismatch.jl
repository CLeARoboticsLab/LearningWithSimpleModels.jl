function unicycle_no_mismatch()
    training_params = TrainingParameters(;
        name = "unicycle",
        save_path = ".data/results/JetRacer_simulation/no_mismatch",
        hidden_layer_sizes = [64, 64],
        learning_rate = 6.00e-3,
        iters = 15,
        optim = gradient_descent,
        loss_aggregation = simulation_timestep,
        plot = true,
        save_model = true,
        save_plot = true,
        save_animation = false,
        save_all_data = true
    )

    train(
        unicycle_simple_dynamics(scale=1.0), 
        unicycle_actual_dynamics();
        controller = unicycle_controller(), 
        cost = unicycle_cost(),
        task = unicycle_figure_eight_task(),
        algo = unicycle_training_algorithm(),
        training_params = training_params,
        sim_params = unicycle_simulation_parameters()
    )

    eval_params = EvaluationParameters(;
        name = "unicycle",
        path = ".data/results/JetRacer_simulation/no_mismatch",    
        n_task_executions = 3,
        save_plot = true,
        save_animation = false
    )

    eval_data = evaluate_model(;
        actual_dynamics = unicycle_actual_dynamics(),
        controller = unicycle_controller(),
        cost = unicycle_cost(),
        task = unicycle_figure_eight_task(),
        algo = unicycle_training_algorithm(),
        training_params = training_params,
        sim_params = unicycle_simulation_parameters(),
        eval_params = eval_params,
        model = nothing
    )
end