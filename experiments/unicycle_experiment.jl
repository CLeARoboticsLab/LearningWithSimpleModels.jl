using LearningWithSimpleModels
using GLMakie, LinearAlgebra

include("unicycle_controller.jl")

struct UnicycleSimpleParameters <: SystemParameters end

unicycle_simple_dynamics() = SimpleDynamics(;
    params = UnicycleSimpleParameters(),
    f = (dyn::SimpleDynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) -> begin
        x_next = similar(x)
        x_next[1] = x[1] + x[3]*cos(x[4])*dt
        x_next[2] = x[2] + x[3]*sin(x[4])*dt
        x_next[3] = x[3] + u[1]*dt
        x_next[4] = x[4] + u[2]*dt
        return x_next
    end
)

Base.@kwdef struct UnicycleActualParameters <: SystemParameters 
    vel_drag::Float64
    turn_drag::Float64
    accel_scale::Float64
    turn_rate_scale::Float64
end

unicycle_actual_dynamics() = ActualDynamics(;
    params = UnicycleActualParameters(;
        vel_drag = 0.5,
        turn_drag = 0.25,
        accel_scale = 0.95,
        turn_rate_scale = 0.95
    ),
    f = (dyn::ActualDynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) -> begin
        x_next = similar(x)
        x_next[1] = x[1] + x[3]*cos(x[4])*dt
        x_next[2] = x[2] + x[3]*sin(x[4])*dt
        x_next[3] = x[3] + (dyn.params.accel_scale*u[1] - dyn.params.vel_drag*x[3])*dt
        x_next[4] = x[4] + (dyn.params.turn_rate_scale*u[2] - dyn.params.turn_drag*x[4])*dt
        return x_next
    end
)

unicycle_cost() = quadratic_cost(;
    Q=diagm(ones(4)),
    R=diagm(ones(2))
)

unicycle_figure_eight_task() = figure_eight(;
    x0 = 0.0,
    y0 = 0.0,
    xdot_0 = 0.0,
    ydot_0 = 0.0,
    xdot_f = nothing,
    ydot_f = nothing,
    radius = 3.0,
    time = 10.0
)

function run()
    train(;
        simple_dynamics = unicycle_simple_dynamics(), 
        actual_dynamics = unicycle_actual_dynamics(),
        controller = unicycle_controller(), 
        cost = unicycle_cost(),
        task = unicycle_figure_eight_task(),
        params = TrainingParameters(;
            n_states = 4,
            dt = 0.01,
            model_dt = 0.5,
            hidden_layer_sizes = [64, 64],
            learning_rate = 1e-3,
            iters = 50
        )
    )
end
