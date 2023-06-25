function export_csvs()
    base_path = ".data/results/JetRacer/training_data/1.00e-3_lr"
    paths = [
        joinpath(base_path, "jetracer_training_data_1.bson"),
        joinpath(base_path, "jetracer_training_data_2.bson"),
        joinpath(base_path, "jetracer_training_data_3.bson")
    ]
    rs = Vector{RolloutData}()
    for path in paths
        run = BSON.load(path)
        append!(rs, run[:data].rollouts)
    end

    base_export_path = ".csv_files/JetRacer/training_data/1.00e-3_lr"
    mkpath(base_export_path)
    for (i,r) in enumerate(rs)
        make_csv(joinpath(base_export_path, "rollout_$(i).csv"), r)
    end
end