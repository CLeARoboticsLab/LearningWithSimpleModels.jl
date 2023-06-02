dp_simple_dynamics() = Dynamics(;
    f = (dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) -> begin
        m1 = 1
        m2 = 1
        g = 9.81
        l1 = 1
        l2 = 1
        
        M = m1 + m2
        Δθ = x[1] - x[2]
        α = m1 + m2*sin(Δθ)^2
        return [
            x[1] + x[3]*dt,
            x[2] + x[4]*dt,
            x[3] + ((l1*α)^(-1)*(-sin(Δθ)*(m2*l1*x[3]^2*cos(Δθ) + m2*l2*x[4]^2) - g*( M*sin(x[1]) - m2*sin(x[2])*cos(Δθ))) + u[1])*dt,
            x[4] + ((l2*α)^(-1)*( sin(Δθ)*(m2*l2*x[4]^2*cos(Δθ) +  M*l1*x[3]^2) + g*(-M*sin(x[2]) +  M*sin(x[1])*cos(Δθ))) + u[2])*dt,
        ]
    end
)

dp_actual_dynamics() = Dynamics(;
    f = (dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) -> begin
        m1 = 1
        m2 = 1
        g = 9.81
        l1 = 1
        l2 = 1
        
        M = m1 + m2
        Δθ = x[1] - x[2]
        α = m1 + m2*sin(Δθ)^2
        return [
            x[1] + x[3]*dt,
            x[2] + x[4]*dt,
            x[3] + ((l1*α)^(-1)*(-sin(Δθ)*(m2*l1*x[3]^2*cos(Δθ) + m2*l2*x[4]^2) - g*( M*sin(x[1]) - m2*sin(x[2])*cos(Δθ))) + u[1])*dt,
            x[4] + ((l2*α)^(-1)*( sin(Δθ)*(m2*l2*x[4]^2*cos(Δθ) +  M*l1*x[3]^2) + g*(-M*sin(x[2]) +  M*sin(x[1])*cos(Δθ))) + u[2])*dt,
        ]
    end
)

Base.@kwdef struct DpCostParameters <: CostParameters
end

dp_cost() = Cost(;
    params = DpCostParameters(),
    g = (cost::Cost, time::Real, x::Vector{Float64}, x_des::Vector{Float64}, cir::FigEightCircle, u::Vector{Float64}) -> begin
        return 0.0 # TODO
    end
)

function dp_fig_eight_task_time_estimate(cir::FigEightCircle, time::Real, x::Vector{Float64})
    center_y = 0.0
    t = wrapped_time(cir,time)
    nom_quadrant = Integer(ceil(t/cir.time * 4))
    if nom_quadrant == 1 || nom_quadrant == 0
        if x[2] > 0.0
            # in quadrant 1
            center_x = cir.r
            arc_angle = atan(x[2]-center_y, x[1]-center_x)
            time_est = cir.time/4*(π-arc_angle)/π
        else
            # in quadrant 4
            center_x = -cir.r
            arc_angle = atan(x[2]-center_y, x[1]-center_x)
            time_est =  3/4*cir.time + cir.time/4*(π+arc_angle)/π
        end
    elseif nom_quadrant == 2
        center_x = cir.r
        arc_angle = atan(x[2]-center_y, x[1]-center_x)
        if x[2] > 0.0
            # in quadrant 1
            time_est = cir.time/4*(π-arc_angle)/π
        else
            # in quadrant 2
            time_est =  1/4*cir.time + cir.time/4*(-arc_angle)/π
        end
    elseif nom_quadrant == 3
        if x[2] > 0.0
            # in quadrant 3
            center_x = -cir.r
            arc_angle = atan(x[2]-center_y, x[1]-center_x)
            time_est =  1/2*cir.time + cir.time/4*(arc_angle)/π
        else
            # in quadrant 2
            center_x = cir.r
            arc_angle = atan(x[2]-center_y, x[1]-center_x)
            time_est =  1/4*cir.time + cir.time/4*(-arc_angle)/π
        end
    elseif nom_quadrant == 4   
        center_x = -cir.r
        arc_angle = atan(x[2]-center_y, x[1]-center_x)
        if x[2] > 0.0
            # in quadrant 3
            time_est =  1/2*cir.time + cir.time/4*(arc_angle)/π
        else
            # in quadrant 4
            time_est =  3/4*cir.time + cir.time/4*(π+arc_angle)/π
        end
    end
    return time_est
end

T() = 5.0
m_dt() = 0.1

dp_figure_eight_task() = FigEightCircle(; r=1.5, time = T() ) # TODO

dp_training_algorithm() = RandomInitialAlgorithm(; # TODO
    variances = [.010^2, .010^2, 0.001^2, .002^2],
    n_rollouts_per_update = 1,
    n_beginning_segs_to_truncate = Integer(round(0.25*T()/(m_dt()))),
    segs_per_rollout = Integer(round((0.25*T() + 1.35*T())/m_dt())),
    segs_in_window = Integer(round(.75*T()/(m_dt()))),
    to_state = (task_point) -> to_velocity_and_heading_angle(task_point),
    task_time_est = nothing
)

dp_training_parameters() = TrainingParameters(; # TODO
    name = "dp_sim",
    save_path = ".data",
    hidden_layer_sizes = [64, 64],
    learning_rate = 4.0e-4,
    iters = 0,
    optim = gradient_descent,
    loss_aggregation = simulation_timestep,
    save_model = true,
    save_plot = true,
    save_animation = false,
    save_all_data = true
)

dp_simulation_parameters() = SimulationParameters(;
    x0 = [π/2-0.1, π/2-0.1, 0.0, 0.0],
    n_inputs = 2,
    dt = 0.01, # should match controller update rate
    model_dt = m_dt(),
    model_scale = [1.0, 1.0, 0.0, 0.0, 1.00, 1.00, 0.025, 0.25, 0.025, 0.35] # TODO
    #                                  kx    ky    kvx   kϕ    kvy   kω
)

dp_evaluation_parameters() = EvaluationParameters(;
    name = "dp_sim",
    path = ".data",    
    n_task_executions = 1,
    save_plot = true,
    save_animation = false
)