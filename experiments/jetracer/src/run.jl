function run()
    
    task = jetracer_figure_eight_task()
    training_params = jetracer_training_parameters()
    sim_params = jetracer_simulation_parameters()
    ctrl_params = jetracer_controller_parameters()
    simple_dynamics = jetracer_simple_dynamics()
    controller = jetracer_controller()
    cost = jetracer_cost()
    n_segments = 10 # TODO make param? (input rollout time in sec)

    p = ProgressMeter.Progress(training_params.iters)
    model = make_model(length(sim_params.x0), training_params.hidden_layer_sizes)
    connections = open_connections()

    for i in 1:training_params.iters
        r = rollout_actual_dynamics(
            connections,
            task,
            model,
            sim_params,
            ctrl_params,
            n_segments
        )
        loss, grads = gradient_estimate(
            r,
            task,
            model,
            simple_dynamics,
            controller,
            cost,
            sim_params
        )
        ProgressMeter.next!(p, showvalues = [(:loss,loss)])
    end

    close_connections(connections)
end