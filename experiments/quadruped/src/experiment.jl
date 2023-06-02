quadruped_simple_dynamics() = Dynamics(;
    f = (dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) -> begin
        return [
            x[1] + u[1]*cos(x[4])*dt,
            x[2] + u[1]*sin(x[4])*dt,
            u[1],
            x[4] + u[2]*dt
        ]
    end
)

Base.@kwdef struct QuadrupedCostParameters <: CostParameters
    vel_weight::Float64 = 0.1
    angle_weight::Float64 = 0.1
    input_weight::Float64 = 0.0
end

quadruped_cost() = Cost(;
    params = QuadrupedCostParameters(; vel_weight=1/30, angle_weight=10/30, input_weight = 0.00),
    g = (cost::Cost, time::Real, x::Vector{Float64}, x_des::Vector{Float64}, 
    cir::FigEightCircle, u::Vector{Float64}, simple_dynamics::Dynamics) -> begin
    
        t = wrapped_time(cir,time)
        extra_weight = 1.0        

        nom_quadrant = Integer(ceil(t/cir.time * 4))

        center_x = 0.0
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
        else
            println("Warning: invalid nom_quadrant: $(nom_quadrant)")
        end

        center_y = 0.0
        r = sqrt((x[1]-center_x)^2 + (x[2]-center_y)^2)

        v = x[3]
        ϕ = x[4]
        arc_angle = atan(x[2]-center_y, x[1]-center_x)

        if nom_quadrant == 1
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
            + cost.params.input_weight*(sum(u.^2))
        )
    end
)

function quadruped_fig_eight_task_time_estimate(cir::FigEightCircle, time::Real, x::Vector{Float64})
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

T() = 2*2*π*1.5/0.5
m_dt() = T()/20.0/2

quadruped_figure_eight_task() = FigEightCircle(; r=1.5, time = T() )

quadruped_training_algorithm() = HardwareTrainingAlgorithm(;
    seconds_per_rollout = 0.25*T() + 1.35*T(),
    n_beginning_segs_to_truncate = Integer(round(0.25*T()/(m_dt()))),
    use_window = true,
    segs_in_window = Integer(round(.75*T()/(m_dt()))),
    stopping_segments = 0,
    task_time_est = quadruped_fig_eight_task_time_estimate
)

quadruped_training_parameters() = TrainingParameters(;
    name = "quadruped",
    save_path = ".data",
    hidden_layer_sizes = [64, 64],
    learning_rate = 2.0e-4,
    iters = 10,
    optim = gradient_descent,
    loss_aggregation = simulation_timestep,
    save_model = true,
    save_plot = true,
    save_animation = true,
    save_all_data = true
)

quadruped_simulation_parameters() = SimulationParameters(;
    x0 = [0.0, 0.0, 0.0, 0.0],
    n_inputs = 2,
    dt = 1.0/10.0, # should match controller update rate
    model_dt = m_dt(),
    model_scale = [1.0, 1.0, 0.0, 0.0, 0.25, 0.25, 1.00, 0.25, 1.00, 0.35]
    #                                  kx    ky    kvx   kϕ    kvy   kω
)

quadruped_evaluation_parameters() = EvaluationParameters(;
    name = "quadruped",
    path = ".data",    
    n_task_executions = 3,
    save_plot = true,
    save_animation = true
)