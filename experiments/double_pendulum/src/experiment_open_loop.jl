dp_actual_dynamics_params_open_loop() = DoublePendulumDynamicsParameters(
    act_m1(), act_m2(), act_l1(), act_l2(), act_g()
)
dp_simple_dynamics_params_open_loop(; mass_mismatch=0.5, length_mismatch=1.0) = DoublePendulumDynamicsParameters(
    mass_mismatch*act_m1(), mass_mismatch*act_m2(), length_mismatch*act_l1(), length_mismatch*act_l2(), act_g()
)

m_dt_open_loop() = 0.01
sim_dt_open_loop() = 0.01

dp_simple_dynamics_open_loop(; mass_mismatch=0.5, length_mismatch=1.0) = Dynamics(;
    params = dp_simple_dynamics_params_open_loop(; mass_mismatch=mass_mismatch, length_mismatch=length_mismatch),
    f = dp_simple_dynamics_f
)

dp_actual_dynamics_open_loop() = Dynamics(;
    params = dp_actual_dynamics_params_open_loop(),
    f = dp_actual_dynamics_f
)

dp_task_open_loop() = ConstantTask([0.0, 0.0, 0.0, 0.0], T())

start_state_open_loop() = [5*π/8, 0.0, 0.0, 0.0]

dp_cost_open_loop() = Cost(;
    params = DpCostParameters(),
    g = (cost::Cost, time::Real, x::Vector{Float64}, x_des::Vector{Float64}, 
    task::ConstantTask, u::Vector{Float64}, simple_dynamics::Dynamics) -> begin
        x_end_eff, y_end_eff = end_effector_position(simple_dynamics.params, x[1], x[2])
        x_task, y_task = const_point(time)
        return (
            (x_end_eff - x_task)^2 + (y_end_eff - y_task)^2
            + 0.0*(sum(u.^2))
        )
    end
)

dp_training_algorithm_open_loop() = RandomInitialAlgorithm(;
    variances = [π/8, 0, 0, 0],
    perc_of_task_to_sample = 0, # starting point is always beginning
    n_rollouts_per_update = 5,
    n_beginning_segs_to_truncate = 0,
    segs_per_rollout = Integer(round(T()/m_dt_open_loop())),
    segs_in_window = Integer(round(T()/m_dt_open_loop())),
    to_state = (task_point) -> start_state_open_loop(), # used to convert task point to state when starting rollout
    task_time_est = nothing
)

dp_training_parameters_open_loop(; save_path=".data") = TrainingParameters(; # TODO
    name = "dp_sim_open_loop",
    save_path = save_path,
    hidden_layer_sizes = [64, 64],
    learning_rate = 4.0e-7,
    iters = 100,
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
    t_transformed = [cos(2*π*t/task_time), sin(2*π*t/task_time)]
    x_transformed = [
        cos(x[1]),
        sin(x[1]),
        cos(x[2]),
        sin(x[2]),
        x[3],
        x[4]
    ]
    return vcat(x_transformed, t_transformed)
end

dp_simulation_parameters_open_loop() = SimulationParameters(;
    x0 = start_state_open_loop(),
    n_inputs = 2,
    dt = sim_dt_open_loop(),
    model_dt = m_dt_open_loop(),
    model_scale = [
        1.0, 
        1.0, 
        0.0, 
        0.0, 
        0.0,
        0.0, 
        0.0,  
        0.0,
        0.0, 
        0.0
    ],
    model_in_dim = 8,
    model_input_function = dp_model_input_function_open_loop,
    spline_seg_type = NoSpline()
)

dp_evaluation_parameters_open_loop() = EvaluationParameters(; #add here
    name = "dp_sim_open_loop",
    type = DoublePendulumEvalType(),
    f = const_point,
    path = ".data",    
    n_task_executions = 1,
    save_plot = true,
    save_animation = true
)