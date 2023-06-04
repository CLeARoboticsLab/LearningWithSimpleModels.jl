dp_task_open_loop() = ConstantTask([0.0, 0.0, 0.0, 0.0], T())

dp_training_algorithm_open_loop() = RandomInitialAlgorithm(;
    variances = [0, 0, 0, 0],
    perc_of_task_to_sample = 0, # starting point is always beginning
    n_rollouts_per_update = 1,
    n_beginning_segs_to_truncate = 0,
    segs_per_rollout = Integer(round(T()/m_dt())),
    segs_in_window = Integer(round(T()/m_dt())),
    to_state = (task_point) -> task_point, # used to convert task point to state
    task_time_est = nothing
)

dp_training_parameters_open_loop() = TrainingParameters(; # TODO
    name = "dp_sim_open_loop",
    save_path = ".data",
    hidden_layer_sizes = [64, 64],
    learning_rate = 1e-6,
    iters = 10000,
    optim = gradient_descent,
    loss_aggregation = simulation_timestep,
    save_model = true,
    save_plot = true,
    save_animation = false,
    save_all_data = true
)

dp_simulation_parameters_open_loop() = SimulationParameters(;
    x0 = [π, 0.0, 0.0, 0.0],
    n_inputs = 2,
    dt = sim_dt(),
    model_dt = sim_dt(),  # TODO: change to sim_dt()?
    model_scale = [
        5.0, 
        5.0, 
        0.0, 
        0.0, 
        0.0,
        0.0, 
        0.0,  
        0.0,
        0.0, 
        0.0
    ],
    model_in_dim = 6,
    model_input_function = dp_model_input_function,
    spline_seg_type = NoSpline()
)

dp_evaluation_parameters_open_loop() = EvaluationParameters(; #add here
    name = "dp_sim_open_loop",
    type = DoublePendulumEvalType(),
    f = star,
    path = ".data",    
    n_task_executions = 1,
    save_plot = true,
    save_animation = true
)