import BSON

mismatch_scales() = [1.0, 0.75, 0.5, 0.25]

function mismatch_study()

    scales = mismatch_scales()
    num_episodes = 10
    iters = 16

    losses = zeros(length(scales), num_episodes, iters)
    K = length(scales) * num_episodes

    training_params = TrainingParameters(;
        name = "unicycle",
        save_path = ".data/results/mismatch_study",
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

    k = 1
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
            println("Finished training $k / $K")
            k += 1
        end
    end

    BSON.@save ".data/results/mismatch_study/mismatch_study.bson" losses
end

function plot_mismatch_study()
    scales = mismatch_scales()
    BSON.@load ".data/results/mismatch_study/mismatch_study.bson" losses
    plot_variances(".data/results/mismatch_study/training.png", scales, losses; xlim=20)
end

function mismatch_car_plots()

    scales = mismatch_scales()
    iters = 20

    r_no_model = Nothing
    rs = Vector{RolloutData}()
    for scale in scales

        name = "unicycle_with_" * string(scale) * "_mismatch"

        training_params = TrainingParameters(;
            name = name,
            save_path = ".data/results/mismatch_study",
            hidden_layer_sizes = [64, 64],
            learning_rate = 6.00e-3,
            iters = iters,
            optim = gradient_descent,
            loss_aggregation = simulation_timestep,
            plot = false,
            save_model = true,
            save_plot = false,
            save_animation = false,
            save_all_data = false
        )

        model, losses = train(
            unicycle_simple_dynamics(scale=scale), 
            unicycle_actual_dynamics();
            controller = unicycle_controller(), 
            cost = unicycle_cost(),
            task = unicycle_figure_eight_task(),
            algo = unicycle_training_algorithm(),
            training_params = training_params,
            sim_params = unicycle_simulation_parameters()
        )

        eval_params = EvaluationParameters(;
            name = name,
            path = ".data/results/mismatch_study",    
            n_task_executions = 3,
            save_plot = false,
            save_animation = false
        )

        eval_data = evaluate_model(;
            actual_dynamics = unicycle_actual_dynamics(),
            controller = unicycle_controller(),
            cost = unicycle_cost(),
            task = unicycle_figure_eight_task(),
            algo = unicycle_training_algorithm(),
            training_params = unicycle_training_parameters(),
            sim_params = unicycle_simulation_parameters(),
            eval_params = eval_params,
            model = model
        )

        r_no_model = eval_data.r_no_model
        push!(rs, eval_data.r)
    end
    final_eval_plot(
        ".data/results/mismatch_study/evals.png", rs, r_no_model, scales,
        unicycle_figure_eight_task(), 800, 350, .33, .66, .33
    )
end