function make_training_plot()
    base_path = ".data/results/JetRacer/training_data/0.75e-3_lr"
    paths = [
        joinpath(base_path, "jetracer_training_data_1.bson"),
        joinpath(base_path, "jetracer_training_data_2.bson"),
        joinpath(base_path, "jetracer_training_data_3.bson")
    ]
    runs = Vector{TrainingData}()
    for path in paths
        run = BSON.load(path)
        push!(runs, run[:data])
    end

    multi_training_plot(runs,".data/results/JetRacer/losses_large.png",800,600)
    multi_training_plot(runs,".data/results/JetRacer/jr_losses_small.png",375,325)
end

function make_training_plot_corl()
    base_path = ".data/results/JetRacer/training_data/1.00e-3_lr"
    paths = [
        joinpath(base_path, "jetracer_training_data_1.bson"),
        joinpath(base_path, "jetracer_training_data_2.bson"),
        joinpath(base_path, "jetracer_training_data_3.bson")
    ]
    runs = Vector{TrainingData}()
    for path in paths
        run = BSON.load(path)
        push!(runs, run[:data])
    end
    LearningWithSimpleModels.multi_training_plot_reward(runs,".data/results/JetRacer/jr_losses_small.png",365,325)
end

function make_final_eval_plot_early()
    base_path = ".data/results/JetRacer/training_data/1.00e-3_lr"
    paths = [
        joinpath(base_path, "jetracer_training_data_2.bson")
    ]
    runs = Vector{TrainingData}()
    for path in paths
        run = BSON.load(path)
        push!(runs, run[:data])
    end

    r = runs[1].rollouts[8]

    base_path = ".data/results/JetRacer/runs/2_1.0lr"
    r_no_model = BSON.load(joinpath(base_path,"jetracer_rollout_no_model.bson"))[:r_no_model]

    task = jetracer_figure_eight_task()
    path = ".data/results/JetRacer/trajectory.png"
    final_eval_plot(path, r, r_no_model, task,  800, 350, .42, .99, 1.6/6.42)
end

function make_final_eval_plot()
    base_path = ".data/results/JetRacer/runs/2_1.0lr"
    r = BSON.load(joinpath(base_path,"jetracer_rollout_using_model.bson"))[:r]
    r_no_model = BSON.load(joinpath(base_path,"jetracer_rollout_no_model.bson"))[:r_no_model]
    task = jetracer_figure_eight_task()
    path = ".data/results/JetRacer/trajectory.png"
    final_eval_plot(path, r, r_no_model, task,  800, 350, .42, .99, 1/6.42)
end

function make_final_eval_plot_corl_fig_1()
    base_path = ".data/results/JetRacer/runs/2_1.0lr"
    r = BSON.load(joinpath(base_path,"jetracer_rollout_using_model.bson"))[:r]
    r_no_model = BSON.load(joinpath(base_path,"jetracer_rollout_no_model.bson"))[:r_no_model]

    base_path = ".data/results/JetRacer_simulation/no_mismatch"
    r_no_mismatch = BSON.load(joinpath(base_path,"unicycle_rollout_using_model.bson"))[:r]
    println("r_no_mismatch")

    task = jetracer_figure_eight_task()
    path = ".data/results/JetRacer/trajectory_with_exact.png"
    final_eval_plot(path, r, r_no_model, r_no_mismatch, task,
                    800, 350, .42, .99, 1/6.42, .35, .99)
end

function make_model_outputs_plot()
    base_path = ".data/results/JetRacer/runs/2_1.0lr"
    r = BSON.load(joinpath(base_path,"jetracer_rollout_using_model.bson"))[:r]
    task = jetracer_figure_eight_task()

    path = ".data/results/JetRacer/model_outputs_one_lap.png"
    final_model_outputs_plot(path, r, task, 
        jetracer_controller_parameters(), [1,2,3,4,0,5],
        1500, 600, 4.77/6, 5.687/6;
        leg=(3,5), gains_color=:black)

    path = ".data/results/JetRacer/model_outputs_one_lap_small.png"
    final_model_outputs_plot(path, r, task, 
        jetracer_controller_parameters(), [1,2,3,4,0,5],
        1250, 500, 4.77/6, 5.687/6;
        leg=(3,5), gains_color=:black)
end

function make_model_outputs_plot_corl()
    base_path = ".data/results/JetRacer/runs/2_1.0lr"
    r = BSON.load(joinpath(base_path,"jetracer_rollout_using_model.bson"))[:r]
    r_no_model = BSON.load(joinpath(base_path,"jetracer_rollout_no_model.bson"))[:r_no_model]
    task = jetracer_figure_eight_task()

    path = ".data/results/JetRacer/jr_model_outputs_and_eval.png"
    LearningWithSimpleModels.final_model_outputs_plot_with_untrained_corl(path, r, task, 
        jetracer_controller_parameters(), [1,2,3,4,0,5],
        1100, 425, 4.77/6, 5.687/6,
        r_no_model, 1/6.42;
        leg=(3,5), gains_color=:black,
    )
end


function make_model_outputs_plot_early()
    base_path = ".data/results/JetRacer/training_data/1.00e-3_lr"
    paths = [
        joinpath(base_path, "jetracer_training_data_2.bson")
    ]
    runs = Vector{TrainingData}()
    for path in paths
        run = BSON.load(path)
        push!(runs, run[:data])
    end
    r = runs[1].rollouts[8]

    base_path = ".data/results/JetRacer/runs/2_1.0lr"
    r_no_model = BSON.load(joinpath(base_path,"jetracer_rollout_no_model.bson"))[:r_no_model]
    task = jetracer_figure_eight_task()
    path = ".data/results/JetRacer/jr_model_outputs_and_eval.png"
    LearningWithSimpleModels.final_model_outputs_plot_with_untrained_corl(path, r, task, 
        jetracer_controller_parameters(), [1,2,3,4,0,5],
        1100, 425, 4.10/6, 5.570/6,
        r_no_model, 1.6/6.42;
        leg=(3,5), gains_color=:black,
    )
end
.42, .99, 1.6/6.42
function make_final_eval_animation()
    base_path = ".data/results/JetRacer/runs/2_1.0lr"
    r = BSON.load(joinpath(base_path,"jetracer_rollout_using_model.bson"))[:r]
    task = jetracer_figure_eight_task()

    path = ".data/results/JetRacer/eval_animation.mp4"
    animate_final_evaluation(
        path, r, task, .475, .99, (Makie.wong_colors()[1], 1.0)
    )

    base_path = ".data/results/JetRacer/runs/2_1.0lr"
    r = BSON.load(joinpath(base_path,"jetracer_rollout_no_model.bson"))[:r_no_model]
    task = jetracer_figure_eight_task()

    path = ".data/results/JetRacer/eval_animation_untrained.mp4"
    animate_final_evaluation(
        path, r, task, .475, .99, (Makie.wong_colors()[2], 1.0)
    )
end