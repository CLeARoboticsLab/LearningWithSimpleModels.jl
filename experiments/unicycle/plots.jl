using BSON

function make_final_eval_plot()
    base_path = ".data/results/JetRacer_simulation/data"
    r = BSON.load(joinpath(base_path,"unicycle_rollout_using_model.bson"))[:r]
    r_no_model = BSON.load(joinpath(base_path,"unicycle_rollout_no_model.bson"))[:r_no_model]
    task = unicycle_figure_eight_task()
    path = ".data/results/JetRacer_simulation/trajectory.png"
    final_eval_plot(path, r, r_no_model, task, 800, 450, 0.20, .99, 1/6.1)
end

function make_model_outputs_plot()
    base_path = ".data/results/JetRacer_simulation/data"
    r = BSON.load(joinpath(base_path,"unicycle_rollout_using_model.bson"))[:r]
    task = unicycle_figure_eight_task()

    path = ".data/results/JetRacer_simulation/model_outputs_one_lap.png"
    final_model_outputs_plot(path, r, task, 
        unicycle_controller_parameters(), [1,2,3,4,0,5],
        1500, 600, 4.976/6, 5.935/6)
end