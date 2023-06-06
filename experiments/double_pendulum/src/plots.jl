function final_variances_plot()
    base_path = ".data/2023-06-05"
    save_path = joinpath(base_path, "variances.png")

    paths1 = [
        joinpath(base_path, "closed_loop_01/dp_sim_training_data.bson"),
        joinpath(base_path, "closed_loop_02/dp_sim_training_data.bson"),
        joinpath(base_path, "closed_loop_03/dp_sim_training_data.bson"),
        joinpath(base_path, "closed_loop_04/dp_sim_training_data.bson"),
        joinpath(base_path, "closed_loop_05/dp_sim_training_data.bson"),
    ]
    runs1 = Vector{TrainingData}()
    for path in paths1
        run = BSON.load(path)
        push!(runs1, run[:data])
    end

    paths2 = [
        joinpath(base_path, "open_loop_01/dp_sim_open_loop_training_data.bson"),
        joinpath(base_path, "open_loop_02/dp_sim_open_loop_training_data.bson"),
        joinpath(base_path, "open_loop_03/dp_sim_open_loop_training_data.bson"),
        joinpath(base_path, "open_loop_04/dp_sim_open_loop_training_data.bson"),
        joinpath(base_path, "open_loop_05/dp_sim_open_loop_training_data.bson"),
    ]
    runs2 = Vector{TrainingData}()
    for path in paths2
        run = BSON.load(path)
        push!(runs2, run[:data])
    end

    plot_variances(save_path, runs1, runs2, 100)
end