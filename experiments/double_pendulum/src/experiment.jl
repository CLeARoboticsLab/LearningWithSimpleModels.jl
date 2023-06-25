struct DoublePendulumDynamicsParameters <: DyanmicsParameters
    m1
    m2
    l1
    l2
    g
end

act_m1() = 0.1
act_m2() = 0.1
act_l1() = 1.0
act_l2() = 1.0
act_g() = 1.0

dp_actual_dynamics_params() = DoublePendulumDynamicsParameters(act_m1(), act_m2(), act_l1(), act_l2(), act_g())
dp_simple_dynamics_params() = DoublePendulumDynamicsParameters(
    0.5*act_m1(), 
    0.5*act_m2(), 
    1.0*act_l1(), 
    1.0*act_l2(), 
    act_g()
)

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
    # x[1] = θ₁; # x[2] = θ₂; x[3] = θ₁_dot; x[4] = θ₂_dot
    
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

# go to a point and stay
# const_point(time) = ((act_l1()+act_l2())*sin(5*π/8), -(act_l1()+act_l2())*cos(5*π/8))
const_point(time) = (1.70, -(act_l1()+act_l2())*cos(5*π/8))

center_x() = 0.35
center_y() = 1.5
radius() = 0.3

# trace a circle
function circle(time)
    reps = 2
    a = reps*2*π/T()
    b = π/2
    x_task = center_x() + radius()*cos(a*time + b)
    y_task = center_y() + radius()*sin(a*time + b)
    return x_task, y_task
end

# trace a star (Hypotrochoid)
function star(time)
    reps = 1
    a = reps*2*π/T()
    c = 0.25*radius()
    d = 0.75*radius()
    x_task = c*sin(3*a*time) - d*sin(2*a*time) + center_x()
    y_task = c*cos(3*a*time) + d*cos(2*a*time) + center_y()
    return x_task, y_task
end

dp_cost() = Cost(;
    params = DpCostParameters(),
    g = (cost::Cost, time::Real, x::Vector{Float64}, x_des::Vector{Float64}, 
    task::ConstantTask, u::Vector{Float64}, simple_dynamics::Dynamics) -> begin
        x_end_eff, y_end_eff = end_effector_position(simple_dynamics.params, x[1], x[2])
        # x_task, y_task = star(time)
        x_task, y_task = const_point(time)
        return (
            (x_end_eff - x_task)^2 + (y_end_eff - y_task)^2
            + 0.0*(sum(u.^2))
        )
    end
)

T() = 3.0
m_dt() = 0.1

dp_task() = ConstantTask([π, 0.0, 0.0, 0.0], T())
start_state() = [5*π/8, 0.0, 0.0, 0.0]

dp_training_algorithm() = RandomInitialAlgorithm(;
    variances = [π/8, 0, 0, 0],
    perc_of_task_to_sample = 0, # starting point is always beginning
    n_rollouts_per_update = 5,
    n_beginning_segs_to_truncate = 0,
    segs_per_rollout = Integer(round(T()/m_dt())),
    segs_in_window = Integer(round(T()/m_dt())),
    to_state = (task_point) -> start_state(), # used to convert task point to state
    task_time_est = nothing
)

dp_training_parameters() = TrainingParameters(; # TODO
    name = "dp_sim",
    save_path = ".data",
    hidden_layer_sizes = [64, 64],
    learning_rate = .25e-4, #.8e-5,
    iters = 100,
    optim = gradient_descent,
    loss_aggregation = simulation_timestep,
    save_model = true,
    save_plot = true,
    save_animation = false,
    save_all_data = true
)

function dp_model_input_function(
    t::Real,
    task_time::Real,
    x::Vector{<:Real},
)
    # # t_transformed = [cos(2*π*t/task_time), sin(2*π*t/task_time)]
    # x_task, y_task = star(t)
    # x_transformed = [
    #     cos(x[1]),
    #     sin(x[1]),
    #     cos(x[2]),
    #     sin(x[2])
    # ]
    # return vcat(x_transformed, [(x_task-center_x())/radius(), (y_task-center_y())/radius()])
    # # return vcat(x_transformed, t_transformed)

    # t_transformed = [cos(2*π*t/task_time), sin(2*π*t/task_time)]
    # x_task, y_task = star(t)
    x_transformed = [
        cos(x[1]),
        sin(x[1]),
        cos(x[2]),
        sin(x[2])
    ]
    return x_transformed
    # return vcat(x_transformed, t_transformed)
end

sim_dt() = 0.01

dp_simulation_parameters() = SimulationParameters(;
    x0 = [5*π/8, 0.0, 0.0, 0.0],
    n_inputs = 2,
    dt = sim_dt(),
    model_dt = m_dt(),
    model_scale = [
        π, 
        π, 
        1.0, 
        1.0, 
        .5*controller_gains()[1,1],
        .5*controller_gains()[2,2], 
        .5*controller_gains()[1,3],  
        .5*controller_gains()[2,4],
        0.0, 
        0.0
    ],
    model_in_dim = 4,
    model_input_function = dp_model_input_function,
    spline_seg_type = NoSpline()
)

dp_evaluation_parameters() = EvaluationParameters(; #add here
    name = "dp_sim",
    type = DoublePendulumEvalType(),
    f = const_point,
    path = ".data",    
    n_task_executions = 1,
    save_plot = true,
    save_animation = true
)