function mismatch_study()

    scales = [0.25, 0.5, 0.75, 1.0]
    num_episodes = 2
    iters = 3

    losses = zeros(length(scales), num_episodes, iters)

    training_params = TrainingParameters(;
        name = "unicycle",
        save_path = ".data",
        hidden_layer_sizes = [64, 64],
        learning_rate = 6.00e-3,
        iters = iters,
        optim = gradient_descent,
        loss_aggregation = simulation_timestep,
        plot = false,
        save_model = false,
        save_plot = false,
        save_animation = false,
        save_all_data = false
    )

    for (i, scale) in enumerate(scales)
        for j in 1:num_episodes
            _, losses[i, j, :] = train(
                unicycle_simple_dynamics(scale=scale), 
                unicycle_actual_dynamics();
                controller = unicycle_controller(), 
                cost = unicycle_cost(),
                task = unicycle_figure_eight_task(),
                algo = unicycle_training_algorithm(),
                training_params = training_params,
                sim_params = unicycle_simulation_parameters()
            )
        end
    end

    display(losses)

end