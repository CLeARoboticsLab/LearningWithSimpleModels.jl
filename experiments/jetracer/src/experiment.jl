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
    input_weight::Float64 = 0.0
end

jetracer_cost() = Cost(;
    params = JetracerCostParameters(; input_weight = 0.01),
    g = (cost::Cost, x::Vector{Float64}, x_des::Vector{Float64}, u::Vector{Float64}) -> begin
        return (
            (x[1] - x_des[1])^2 + (x[2] - x_des[2])^2
            + cost.params.input_weight*(sum(u.^2))
        )
    end
)

jetracer_figure_eight_task() = figure_eight(;
    x0 = 0.0,
    y0 = 0.0,
    xdot_0 = nothing,
    ydot_0 = nothing,
    xdot_f = nothing,
    ydot_f = nothing,
    radius = 1.0,
    time = 10.0,
    laps = 1
)