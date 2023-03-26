jetracer_simple_dynamics() = Dynamics(;
    f = (dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) -> begin
        return [
            x[1] + x[3]*cos(x[4])*dt,
            x[2] + x[3]*sin(x[4])*dt,
            x[3] + u[1]*dt,
            x[4] + u[2]*dt
        ]
    end
)

Base.@kwdef struct JetracerCostParameters <: CostParameters
    vel_weight::Float64 = 0.1
    input_weight::Float64 = 0.0
end

# jetracer_cost() = Cost(;
#     params = JetracerCostParameters(; input_weight = 0.01),
#     g = (cost::Cost, x::Vector{Float64}, x_des::Vector{Float64}, u::Vector{Float64}) -> begin
#         # w = 2.0
#         θ_des = atan(x_des[4], x_des[3])
#         return (
#             (x[1] - x_des[1])^2 + (x[2] - x_des[2])^2 
#             #+ 0.25*(x[3] - sqrt(x_des[3]^2 + x_des[4]^2))^2
#             #+ 0.25*(x[4] - θ_des)^2
#             + cost.params.input_weight*(sum(u.^2))
#             # (x[1] - x_des[1])^2*(1+w*(cos(θ))^2) + (x[2] - x_des[2])^2*(1+w*(sin(θ))^2)
#             # + cost.params.input_weight*(sum(u.^2))
#         )
#     end
# )

jetracer_cost() = Cost(;
    params = JetracerCostParameters(; vel_weight=1/30, input_weight = 0.00),
    g = (cost::Cost, time::Real, x::Vector{Float64}, x_des::Vector{Float64}, cir::FigEightCircle, u::Vector{Float64}) -> begin
        t = wrapped_time(cir,time)
        # center_x = t < cir.time/2 ? cir.r : -cir.r
        center_x = x[1] > 0.0 ? cir.r : -cir.r
        center_y = 0.0
        r = sqrt((x[1]-center_x)^2 + (x[2]-center_y)^2)
        v = x[3]
        return (
            (r - cir.r)^2
            + cost.params.vel_weight*(v - cir.v)^2
            + cost.params.input_weight*(sum(u.^2))
        )
    end
)

Base.@kwdef struct JetracerTerminalCostParameters <: CostParameters
    weight::Float64 = 0.0
end

jetracer_terminal_cost() = Cost(;
    params = JetracerCostParameters(; vel_weight=0.05, input_weight = 0.00),
    g = (cost::Cost, time::Real, x::Vector{Float64}, x_des::Vector{Float64}, cir::FigEightCircle, u::Vector{Float64}) -> begin
        return 0.0
    end
)

# jetracer_figure_eight_task() = figure_eight(;
#     x0 = 0.0,
#     y0 = 0.0,
#     xdot_0 = nothing,
#     ydot_0 = nothing,
#     xdot_f = nothing,
#     ydot_f = nothing,
#     radius = 1.50,
#     time = 6.0,
#     laps = 1
# )

jetracer_figure_eight_task() = FigEightCircle(; r=1.5, time = 5.5)

Base.@kwdef struct HardwareTrainingAlgorithm <:TrainingAlgorithm
    seconds_per_rollout::Float64 = 12.0
    n_beginning_segs_to_truncate::Integer = 0
    use_window::Bool = false
    segs_in_window::Integer = 10
    stopping_segments = 0
end
Base.show(io::IO, p::HardwareTrainingAlgorithm) = print(io,
    "Hardware Training Algorithm
    Seconds per rollout: $(p.seconds_per_rollout)
    Skipped beginning segments: $(p.n_beginning_segs_to_truncate)
    Use window: $(p.use_window)
    Segments in window: $(p.segs_in_window)"
)

jetracer_training_algorithm() = HardwareTrainingAlgorithm(;
    seconds_per_rollout = 5 + 5.5*3.00,
    n_beginning_segs_to_truncate = 20,
    use_window = true,
    segs_in_window = 15,
    stopping_segments = 4
)

jetracer_training_parameters() = TrainingParameters(;
    name = "jetracer",
    save_path = ".data",
    hidden_layer_sizes = [64, 64],
    learning_rate = 1.0e-3,
    iters = 10,
    optim = gradient_descent,
    loss_aggregation = simulation_timestep,
    save_model = true,
    save_plot = true,
    save_animation = true
)

jetracer_simulation_parameters() = SimulationParameters(;
    x0 = [0.0, 0.0, 0.0, 0.0],
    n_inputs = 2,
    dt = 1.0/50.0, # should match controller update rate
    model_dt = 5.5/20.0,
    model_scale = [1.0, 1.0, 1.0, 1.0, 1.0, 0.25, 0.25, 0.25, 0.07]
)

jetracer_evaluation_parameters() = EvaluationParameters(;
    name = "jetracer",
    path = ".data",    
    n_task_executions = 3,
    save_plot = true,
    save_animation = true
)