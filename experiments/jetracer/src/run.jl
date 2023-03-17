function run()
    
    task = jetracer_figure_eight_task()
    algo = jetracer_training_algorithm()
    training_params = jetracer_training_parameters()
    sim_params = jetracer_simulation_parameters()
    ctrl_params = jetracer_controller_parameters()
    simple_dynamics = jetracer_simple_dynamics()
    controller = jetracer_controller()
    cost = jetracer_cost()

    train(;
        simple_dynamics = simple_dynamics,
        controller = controller,
        ctrl_params = ctrl_params,
        cost = cost,
        task = task,
        algo = algo,
        training_params = training_params,
        sim_params = sim_params
    )
end