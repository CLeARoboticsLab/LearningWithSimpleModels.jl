function run()
    
    task = jetracer_figure_eight_task()
    training_params = jetracer_training_parameters()
    sim_params = jetracer_simulation_parameters()
    ctrl_params = jetracer_controller_parameters()
    n_segments = 3 # TODO make param?

    model = make_model(length(sim_params.x0), training_params.hidden_layer_sizes)
    connections = open_connections()

    for _ in 1:3
        rollout_actual_dynamics(
            connections,
            task,
            model,
            sim_params,
            ctrl_params,
            n_segments
        )
    end

    close_connections(connections)
end