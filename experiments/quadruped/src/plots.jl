function make_training_plot()
    base_path = ".data/results/quadruped_preliminary/data"
    paths = [
        joinpath(base_path, "quadruped_training_data.bson")
    ]
    runs = Vector{TrainingData}()
    for path in paths
        run = BSON.load(path)
        push!(runs, run[:data])
    end

    multi_training_plot(runs,".data/results/quadruped_preliminary/losses_large.png",800,600)
    multi_training_plot(runs,".data/results/quadruped_preliminary/losses_small.png",600,400)
end

function make_final_eval_plot()
    base_path = ".data/results/quadruped_preliminary/data"
    r = BSON.load(joinpath(base_path,"quadruped_rollout_using_model.bson"))[:r]
    r_no_model = BSON.load(joinpath(base_path,"quadruped_rollout_no_model.bson"))[:r_no_model]
    task = quadruped_figure_eight_task()
    path = ".data/results/quadruped_preliminary/trajectory.png"
    final_eval_plot(path, r, r_no_model, task, 1000, 450, 0.020, .99, .98/3.0)
end

function make_model_outputs_plot()
    base_path = ".data/results/quadruped_preliminary/data"
    r = BSON.load(joinpath(base_path,"quadruped_rollout_using_model.bson"))[:r]
    task = quadruped_figure_eight_task()

    path = ".data/results/quadruped_preliminary/model_outputs_one_lap.png"
    final_model_outputs_plot(path, r, task, 
        quadruped_controller_parameters(), [1,2,4,3,5,6],
        1500, 500, 1.99/3, 2.95/3)
end