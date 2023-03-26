using LearningWithSimpleModels

include("unicycle_controller.jl")

unicycle_simple_dynamics() = Dynamics(;
    f = (dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) -> begin
        return [
            x[1] + x[3]*cos(x[4])*dt,
            x[2] + x[3]*sin(x[4])*dt,
            x[3] + u[1]*dt,
            x[4] + u[2]*dt
        ]
    end
)

Base.@kwdef struct UnicycleActualParameters <: DyanmicsParameters 
    vel_drag::Float64
    turn_drag::Float64
    accel_scale::Float64
    turn_rate_scale::Float64
end

unicycle_actual_dynamics() = Dynamics(;
    params = UnicycleActualParameters(;
        vel_drag = 0.5,
        turn_drag = 0.0,
        accel_scale = 0.75,
        turn_rate_scale = 0.75
    ),
    f = (dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) -> begin
        return [
            x[1] + x[3]*cos(x[4])*dt,
            x[2] + x[3]*sin(x[4])*dt,
            x[3] + (dyn.params.accel_scale*u[1] - dyn.params.vel_drag*x[3])*dt,
            x[4] + (dyn.params.turn_rate_scale*u[2] - dyn.params.turn_drag*x[4])*dt
        ]
    end
)

Base.@kwdef struct UnicycleCostParameters <: CostParameters #TODO vel weight
    input_weight::Float64 = 0.0
end

# unicycle_cost() = Cost(;
#     params = UnicycleCostParameters(; input_weight = 0.00),
#     g = (cost::Cost, x::Vector{Float64}, x_des::Vector{Float64}, u::Vector{Float64}) -> begin
#         return (
#             (x[1] - x_des[1])^2 + (x[2] - x_des[2])^2
#             + cost.params.input_weight*(sum(u.^2))
#         )
#     end
# )

unicycle_cost() = Cost(;
    params = UnicycleCostParameters(; input_weight = 0.01),
    g = (cost::Cost, time::Real, x::Vector{Float64}, x_des::Vector{Float64}, cir::FigEightCircle, u::Vector{Float64}) -> begin
        t = wrapped_time(cir,time)
        center_x = t < cir.time/2 ? cir.r : -cir.r
        center_y = 0.0
        r = sqrt((x[1]-center_x)^2 + (x[2]-center_y)^2)
        v = x[3]
        return (
            20*(r - cir.r)^2
            + (v - cir.v)^2
            + cost.params.input_weight*(sum(u.^2))
        )
    end
)

# unicycle_figure_eight_task() = figure_eight(;
#     x0 = 0.0,
#     y0 = 0.0,
#     xdot_0 = nothing,
#     ydot_0 = nothing,
#     xdot_f = nothing,
#     ydot_f = nothing,
#     radius = 3.0,
#     time = 10.0,
#     laps = 1
# )

unicycle_figure_eight_task() = FigEightCircle(; r=3.0, time = 10.0)

unicycle_training_algorithm() = WalkingWindowAlgorithm(;
    segs_per_rollout = 60,    
    segs_in_window = 5
)

unicycle_training_algorithm() = RandomInitialAlgorithm(;
    variances = [.25^2, .25^2, .25^2, .25^2],
    n_rollouts_per_update = 3,
    segs_per_rollout = 20,
    segs_in_window = 20,
    to_state = (task_point) -> to_velocity_and_heading_angle(task_point)
)

unicycle_training_parameters() = TrainingParameters(;
    name = "unicycle",
    save_path = ".data",
    hidden_layer_sizes = [64, 64],
    learning_rate = 1.0e-5,
    iters = 10,
    optim = gradient_descent,
    loss_aggregation = simulation_timestep,
    save_model = true,
    save_plot = true,
    save_animation = true
)

unicycle_simulation_parameters() = SimulationParameters(;
    x0 = [0.0, 0.0, 0.0, 0.0],
    n_inputs = 2,
    dt = 0.01,
    model_dt = 0.5,
    model_scale = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
)

unicycle_evaluation_parameters() = EvaluationParameters(;
    name = "unicycle",
    path = ".data",    
    n_task_executions = 3,
    save_plot = true,
    save_animation = true
)

function train_unicycle_experiment()
    model, losses = train(;
        simple_dynamics = unicycle_simple_dynamics(), 
        actual_dynamics = unicycle_actual_dynamics(),
        controller = unicycle_controller(), 
        cost = unicycle_cost(),
        task = unicycle_figure_eight_task(),
        algo = unicycle_training_algorithm(),
        training_params = unicycle_training_parameters(),
        sim_params = unicycle_simulation_parameters()
    )
end

function evaluate_unicycle_experiment()
    eval_data = evaluate_model(;
        actual_dynamics = unicycle_actual_dynamics(),
        controller = unicycle_controller(),
        cost = unicycle_cost(),
        task = unicycle_figure_eight_task(),
        algo = unicycle_training_algorithm(),
        training_params = unicycle_training_parameters(),
        sim_params = unicycle_simulation_parameters(),
        eval_params = unicycle_evaluation_parameters(),
        model = nothing
    )
end
