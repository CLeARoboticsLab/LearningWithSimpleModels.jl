struct DoublePendulumDynamicsParameters <: DyanmicsParameters
    m1
    m2
    l1
    l2
    g
end

dp_actual_dynamics_params() = DoublePendulumDynamicsParameters(1, 1, 1, 1, 9.81)
dp_simple_dynamics_params() = dp_actual_dynamics_params()

H(q, p::DoublePendulumDynamicsParameters) = [
    (p.m1 + p.m2)*p.l1^2 + p.m2*p.l2^2 + 2*p.m2*p.l1*p.l2*cos(q[2])     p.m2*p.l2^2 + p.m2*p.l1*p.l2*cos(q[2])
    p.m2*p.l2^2 + p.m2*p.l1*p.l2*cos(q[2])                              p.m2*p.l2^2
]

C(q, qdot, p::DoublePendulumDynamicsParameters) = [
    0                                   -p.m2*p.l1*p.l2*(2*qdot[1] + qdot[2])*sin(q[2])
    p.m2*p.l1*p.l2*qdot[1]*sin(q[2])    0
]

G(q, p::DoublePendulumDynamicsParameters) = p.g * [
    (p.m1 + p.m2)*p.l1*sin(q[1]) + p.m2*p.l2*sin(q[1]+q[2])
    p.m2*p.l2*sin(q[1]+q[2])
]

B(q, p::DoublePendulumDynamicsParameters) = [
    1   0
    0   1
]

function dp_actual_dynamics_f(dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64})
    # x[1] = θ₁
    # x[2] = θ₂
    # x[3] = θ₁_dot
    # x[4] = θ₂_dot
    
    q = [ x[1], x[2] ]
    qdot = [ x[3], x[4] ]
    p = dyn.params

    x1dot = x[3]
    x2dot = x[4]

    qddot = inv(H(q,p))*( B(q,p)*u - C(q,qdot,p)*qdot - G(q,p) )
    x3dot = qddot[1]
    x4dot = qddot[2]

    return [ # euler integration
        x[1] + x1dot*dt
        x[2] + x2dot*dt
        x[3] + x3dot*dt
        x[4] + x4dot*dt
    ]
end

dp_simple_dynamics_f(dyn::Dynamics, t::Float64, dt::Float64, 
    x::Vector{Float64}, u::Vector{Float64}) = dp_actual_dynamics_f(dyn,t,dt,x,u)

dp_simple_dynamics() = Dynamics(;
    params = dp_simple_dynamics_params(),
    f = dp_simple_dynamics_f
)

dp_actual_dynamics() = Dynamics(;
    params = dp_actual_dynamics_params(),
    f = dp_actual_dynamics_f
)

Base.@kwdef struct DpCostParameters <: CostParameters
end

dp_cost() = Cost(;
    params = DpCostParameters(),
    g = (cost::Cost, time::Real, x::Vector{Float64}, x_des::Vector{Float64}, 
    task::ConstantTask, u::Vector{Float64}, simple_dynamics::Dynamics) -> begin
        
        p = simple_dynamics.params
        x_end_eff = p.l1*sin(x[1]) + p.l2*sin(x[1] + x[2])
        y_end_eff = -p.l1*cos(x[1]) - p.l2*cos(x[1] + x[2])
    
        return 0.0 # TODO
    end
)

T() = 10.0
m_dt() = 0.1

dp_task() = ConstantTask([π, 0.0, 0.0, 0.0], T())

dp_training_algorithm() = RandomInitialAlgorithm(;
    variances = [.001^2, .001^2, 0.001^2, .001^2],
    n_rollouts_per_update = 1,
    n_beginning_segs_to_truncate = 0,
    segs_per_rollout = Integer(round(T()/m_dt())),
    segs_in_window = Integer(round(T()/m_dt())),
    to_state = (task_point) -> task_point, # used to convert task point to state
    task_time_est = nothing
)

dp_training_parameters() = TrainingParameters(; # TODO
    name = "dp_sim",
    save_path = ".data",
    hidden_layer_sizes = [64, 64],
    learning_rate = 4.0e-4,
    iters = 1,
    optim = gradient_descent,
    loss_aggregation = simulation_timestep,
    save_model = true,
    save_plot = true,
    save_animation = false,
    save_all_data = true
)

dp_simulation_parameters() = SimulationParameters(;
    x0 = [0.0, π/2, 0.0, 0.0],
    n_inputs = 2,
    dt = 0.01, # should match controller update rate
    model_dt = m_dt(),
    model_scale = [1.0, 1.0, 0.0, 0.0, 1.00, 1.00, 0.025, 0.25, 0.025, 0.35] # TODO
    #                                  kx    ky    kvx   kϕ    kvy   kω
)

dp_evaluation_parameters() = EvaluationParameters(; #add here
    name = "dp_sim",
    type = DoublePendulumEvalType(),
    path = ".data",    
    n_task_executions = 1,
    save_plot = true,
    save_animation = true
)