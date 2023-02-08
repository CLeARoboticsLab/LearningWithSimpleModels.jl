using LearningWithSimpleModels
using GLMakie, LinearAlgebra

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
        turn_drag = 0.25,
        accel_scale = 0.95,
        turn_rate_scale = 0.95
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

Base.@kwdef struct UnicycleCostParameters <: CostParameters
    input_weight::Float64 = 0.0
end

unicycle_cost() = Cost(;
    params = UnicycleCostParameters(; input_weight = 0.0),
    g = (cost::Cost, x::Vector{Float64}, x_des::Vector{Float64}, u::Vector{Float64}) -> begin
        return (
            (x[1] - x_des[1])^2 + (x[2] - x_des[2])^2
            + cost.params.input_weight*(sum(u.^2))
        )
    end
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

function run_experiment()
    losses = train(;
        simple_dynamics = unicycle_simple_dynamics(), 
        actual_dynamics = unicycle_actual_dynamics(),
        controller = unicycle_controller(), 
        cost = unicycle_cost(),
        task = unicycle_figure_eight_task(),
        params = TrainingParameters(;
            x0 = zeros(4),
            n_inputs = 2,
            dt = 0.01,
            model_dt = 0.5,
            hidden_layer_sizes = [64, 64],
            learning_rate = 1e-3,
            iters = 50,
            segs_in_window = 5
        )
    )

    fig = Figure()
    ax = Axis(fig[1,1])
    lines!(ax, losses)
    display(GLMakie.Screen(), fig)
end

# run_experiment()