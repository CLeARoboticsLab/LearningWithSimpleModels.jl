function final_variances_plot()
    episodes = 3
    
    base_path = ".data/results_pend/no_mismatch"
    save_path = joinpath(base_path, "variances.png")

    paths1 = [joinpath(base_path, "closed_loop_0$(i)/dp_sim_training_data.bson") for i in 1:episodes]
    runs1 = Vector{TrainingData}()
    for path in paths1
        run = BSON.load(path)
        push!(runs1, run[:data])
    end

    paths2 = [joinpath(base_path, "open_loop_0$(i)/dp_sim_open_loop_training_data.bson") for i in 1:episodes]
    runs2 = Vector{TrainingData}()
    for path in paths2
        run = BSON.load(path)
        push!(runs2, run[:data])
    end

    plot_variances(save_path, runs1, runs2, 100, -5000, "")
end

function final_variances_plot_mismatch()
    episodes = 3

    base_path = ".data/results_pend/mismatch"
    save_path = joinpath(base_path, "variances_mismatch.png")
    
    paths1 = [joinpath(base_path, "closed_loop_0$(i)/dp_sim_training_data.bson") for i in 1:episodes]
    runs1 = Vector{TrainingData}()
    for path in paths1
        run = BSON.load(path)
        push!(runs1, run[:data])
    end

    paths2 = [joinpath(base_path, "open_loop_0$(i)/dp_sim_open_loop_training_data.bson") for i in 1:episodes]
    runs2 = Vector{TrainingData}()
    for path in paths2
        run = BSON.load(path)
        push!(runs2, run[:data])
    end

    plot_variances(save_path, runs1, runs2, 100, -5000, "")
end