using BSON

function make_final_eval_plot()
    base_path = ".data/results/simulation/data"
    r = BSON.load(joinpath(base_path,"unicycle_rollout_using_model.bson"))[:r]
    r_no_model = BSON.load(joinpath(base_path,"unicycle_rollout_no_model.bson"))[:r_no_model]
    task = unicycle_figure_eight_task()
    path = ".data/results/simulation/trajectory.png"
    final_eval_plot(path, r, r_no_model, task, 800, 450, 0.20, .99, 1/6.1)
end