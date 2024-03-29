using LearningWithSimpleModels

include("unicycle_controller.jl")

Base.@kwdef struct UnicycleActualParameters <: DyanmicsParameters 
    vel_drag::Float64
    accel_scale::Float64
    turn_rate_scale::Float64
    turn_rate_bias::Float64
end

actual_params() = UnicycleActualParameters(;
    vel_drag = 0.55,
    accel_scale = 11.25,
    turn_rate_scale = 6.50,
    turn_rate_bias = -0.1,
)

unicycle_actual_dynamics() = Dynamics(;
    params = actual_params(),
    f = (dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) -> begin
        return [
            x[1] + x[3]*cos(x[4])*dt,
            x[2] + x[3]*sin(x[4])*dt,
            x[3] + (dyn.params.accel_scale*u[1] - dyn.params.vel_drag*x[3]^2)*dt,
            x[4] + (dyn.params.turn_rate_scale*u[2] - dyn.params.turn_rate_bias)*x[3]*dt
        ]
    end
)

unicycle_simple_dynamics(; scale::Real = 1.0) = Dynamics(;
    params = actual_params(),
    f = (dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) -> begin
        return [
            x[1] + x[3]*cos(x[4])*dt,
            x[2] + x[3]*sin(x[4])*dt,
            x[3] + (scale*dyn.params.accel_scale*u[1] - scale*dyn.params.vel_drag*x[3]^2)*dt,
            x[4] + (scale*dyn.params.turn_rate_scale*u[2] - scale*dyn.params.turn_rate_bias)*x[3]*dt
        ]
    end
)

Base.@kwdef struct UnicycleCostParameters <: CostParameters
    vel_weight::Float64 = 0.1
    angle_weight::Float64 = 0.1
    input_weight::Float64 = 0.0
end

unicycle_cost() = Cost(;
    params = UnicycleCostParameters(; vel_weight=1/30, angle_weight=10/30, input_weight = 0.000),
    g = (cost::Cost, time::Real, x::Vector{Float64}, x_des::Vector{Float64}, 
    cir::FigEightCircle, u::Vector{Float64}, simple_dynamics::Dynamics) -> begin
    
        t = wrapped_time(cir,time)
        extra_weight = 1.0        

        nom_quadrant = Integer(ceil(t/cir.time * 4))

        if nom_quadrant == 1 || nom_quadrant == 0
            if x[2] > 0.0
                center_x = cir.r
            else
                center_x = -cir.r
            end
        elseif nom_quadrant == 2
            center_x = cir.r
        elseif nom_quadrant == 3
            if x[2] > 0.0
                center_x = -cir.r
            else
                center_x = cir.r
            end
        elseif nom_quadrant == 4   
            center_x = -cir.r
        end

        center_y = 0.0
        r = sqrt((x[1]-center_x)^2 + (x[2]-center_y)^2)

        v = x[3]
        ϕ = x[4]
        arc_angle = atan(x[2]-center_y, x[1]-center_x)

        if nom_quadrant == 1 || nom_quadrant == 0
            if x[2] > 0.0
                ϕ_des = arc_angle - π/2
            else
                ϕ_des = arc_angle + π/2
            end
        elseif nom_quadrant == 2
            ϕ_des = arc_angle - π/2  
        elseif nom_quadrant == 3
            if x[2] > 0.0
                ϕ_des = arc_angle + π/2
            else
                ϕ_des = arc_angle - π/2
            end
        elseif nom_quadrant == 4   
            ϕ_des = arc_angle + π/2 
        end

        if abs(ϕ_des - ϕ) > abs(ϕ_des - 2*π - ϕ)
            ϕ_des -= 2*π
        elseif abs(ϕ_des - ϕ) > abs(ϕ_des + 2*π - ϕ)
            ϕ_des += 2*π
        end

        return (
            extra_weight*(r - cir.r)^2
            + cost.params.vel_weight*(v - cir.v)^2
            + cost.params.angle_weight*(ϕ - ϕ_des)^2
            # + cost.params.input_weight*(sum(u.^2))
            + cost.params.input_weight*(u[1]^2)
        )
    end
)

function unicycle_fig_eight_task_time_estimate(cir::FigEightCircle, time::Real, x::Vector{Float64})
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

unicycle_figure_eight_task() = FigEightCircle(; r=1.5, time = 5.5)

# unicycle_training_algorithm() = WalkingWindowAlgorithm(;
#     segs_per_rollout = 60,    
#     segs_in_window = 5
# )

unicycle_training_algorithm() = RandomInitialAlgorithm(;
    variances = [.10^2, .10^2, 0.01^2, .02^2],
    n_rollouts_per_update = 1,
    n_beginning_segs_to_truncate = 20*2,
    segs_per_rollout = 156,
    segs_in_window = 156 - 20*2,
    to_state = (task_point) -> to_velocity_and_heading_angle(task_point),
    task_time_est = unicycle_fig_eight_task_time_estimate
)

unicycle_training_parameters() = TrainingParameters(;
    name = "unicycle",
    save_path = ".data",
    hidden_layer_sizes = [64, 64],
    learning_rate = 6.00e-3,
    iters = 15,
    optim = gradient_descent,
    loss_aggregation = simulation_timestep,
    save_model = true,
    save_plot = true,
    save_animation = true
)

unicycle_simulation_parameters() = SimulationParameters(;
    x0 = [0.0, 0.0, 0.0, π/2],
    n_inputs = 2,
    dt = 1.0/50.0,
    model_dt = 5.5/20.0/2,
    model_scale = [1.0, 1.0, 1.0, 1.0, 0.25, 0.25, 0.25, 0.1, 0.10, 0.02]
)

unicycle_evaluation_parameters() = EvaluationParameters(;
    name = "unicycle",
    path = ".data",    
    n_task_executions = 6,
    save_plot = true,
    save_animation = true
)

function train_unicycle_experiment()
    model, losses = train(
        unicycle_simple_dynamics(scale=0.5), 
        unicycle_actual_dynamics();
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
