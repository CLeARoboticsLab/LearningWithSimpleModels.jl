function make_training_plot()
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

    multi_training_plot(runs,".data/results/JetRacer/losses_large.png",800,600)
    multi_training_plot(runs,".data/results/JetRacer/losses_small.png",600,400)
end

function make_final_eval_plot()
    base_path = ".data/results/JetRacer/runs/2_1.0lr"
    r = BSON.load(joinpath(base_path,"jetracer_rollout_using_model.bson"))[:r]
    r_no_model = BSON.load(joinpath(base_path,"jetracer_rollout_no_model.bson"))[:r_no_model]
    task = jetracer_figure_eight_task()
    path = ".data/results/JetRacer/trajectory.png"
    final_eval_plot(path, r, r_no_model, task, 800, 450, .42, .99, 1/6.42)
end

function make_model_outputs_plot()
    base_path = ".data/results/JetRacer/runs/2_1.0lr"
    r = BSON.load(joinpath(base_path,"jetracer_rollout_using_model.bson"))[:r]
    task = jetracer_figure_eight_task()

    path = ".data/results/JetRacer/model_outputs_one_lap.png"
    final_model_outputs_plot(path, r, task, 
        jetracer_controller_parameters(), [1,2,3,4,0,5],
        1500, 600, 4.77/6, 5.687/6)
end