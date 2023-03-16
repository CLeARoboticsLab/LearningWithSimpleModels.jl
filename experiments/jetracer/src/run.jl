function run()
    
    task = jetracer_figure_eight_task()
    training_params = jetracer_training_parameters()
    sim_params = jetracer_simulation_parameters()
    n_segments = 20 # TODO make param?

    model = make_model(length(sim_params.x0), training_params.hidden_layer_sizes)
    connections = open_connections()

    rollout_actual_dynamics(
        connections,
        task,
        model,
        sim_params,
        n_segments
    )

    close_connections(connections)
end