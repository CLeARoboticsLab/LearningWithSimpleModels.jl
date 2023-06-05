dp_actual_dynamics_params_open_loop() = DoublePendulumDynamicsParameters(act_m1(), act_m2(), act_l1(), act_l2(), 0.0)
dp_simple_dynamics_params_open_loop() = dp_actual_dynamics_params_open_loop()

dp_simple_dynamics_open_loop() = Dynamics(;
    params = dp_simple_dynamics_params_open_loop(),
    f = dp_simple_dynamics_f
)

dp_actual_dynamics_open_loop() = Dynamics(;
    params = dp_actual_dynamics_params_open_loop(),
    f = dp_actual_dynamics_f
)

dp_task_open_loop() = ConstantTask([0.0, 0.0, 0.0, 0.0], T())

start_state_open_loop() = [π, 0.0, 0.0, 0.0]

dp_cost_open_loop() = Cost(;
    params = DpCostParameters(),
    g = (cost::Cost, time::Real, x::Vector{Float64}, x_des::Vector{Float64}, 
    task::ConstantTask, u::Vector{Float64}, simple_dynamics::Dynamics) -> begin
        return (
            (x[1] - 7*π/8)^2 + (x[2] - π/8)^2
        )
    end
)

dp_training_algorithm_open_loop() = RandomInitialAlgorithm(;
    variances = [0, 0, 0, 0],
    perc_of_task_to_sample = 0, # starting point is always beginning
    n_rollouts_per_update = 1,
    n_beginning_segs_to_truncate = 0,
    segs_per_rollout = Integer(round(T()/sim_dt())),
    segs_in_window = Integer(round(T()/sim_dt())),
    to_state = (task_point) -> start_state_open_loop(), # used to convert task point to state when starting rollout
    task_time_est = nothing
)

dp_training_parameters_open_loop() = TrainingParameters(; # TODO
    name = "dp_sim_open_loop",
    save_path = ".data",
    hidden_layer_sizes = [64, 64],
    learning_rate = 2.5e-12,
    iters = 5000,
    optim = gradient_descent,
    loss_aggregation = simulation_timestep,
    save_model = true,
    save_plot = true,
    save_animation = false,
    save_all_data = true
)

function dp_model_input_function_open_loop(
    t::Real,
    task_time::Real,
    x::Vector{<:Real},
)
    # t_transformed = [cos(2*π*t/task_time), sin(2*π*t/task_time)]
    x_task, y_task = star(t)
    x_transformed = [
        cos(x[1]),
        sin(x[1]),
        cos(x[2]),
        sin(x[2])
    ]
    return x_transformed
    # return vcat(x_transformed, [(x_task-center_x())/radius(), (y_task-center_y())/radius()])
end

dp_simulation_parameters_open_loop() = SimulationParameters(;
    x0 = start_state_open_loop(),
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
    model_in_dim = 4,
    model_input_function = dp_model_input_function_open_loop,
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