Base.@kwdef struct UnicycleControllerParameters <: ControllerParameters
    kx::Float64
    ky::Float64
    kv::Float64
    kϕ::Float64
end

function unicycle_policy(controller::Controller, state::Vector{Float64}, setpoints::Vector{Float64})
    x = state[1]
    y = state[2]
    v = state[3]
    ϕ = state[4]

    x_des = setpoints[1]
    y_des = setpoints[2]
    xdot_des = setpoints[3]
    ydot_des = setpoints[4]

    xdot_tilde_des = xdot_des + controller.params.kx*(x_des - x)
    ydot_tilde_des = ydot_des + controller.params.ky*(y_des - y)
    v_des = sqrt(xdot_tilde_des^2 + ydot_tilde_des^2)

    if xdot_tilde_des == 0
        ϕ_des = sign(ydot_tilde_des)*π/2
    else
        ϕ_des = atan(ydot_tilde_des/xdot_tilde_des)
    end

    if xdot_tilde_des < 0.0
        ϕ_des += π
    elseif ydot_tilde_des < 0.0
        ϕ_des += 2*π
    end

    if abs(ϕ_des - ϕ) > abs(ϕ_des - 2*π - ϕ)
        ϕ_des -= 2*π
    end

    a = controller.params.kv*(v_des - v)
    ω = controller.params.kϕ*(ϕ_des - ϕ)

    return [a, ω]
end

unicycle_controller() = Controller(;
    params = UnicycleControllerParameters(;
        kx = 3.0,
        ky = 3.0,
        kv = 3.0,
        kϕ = 3.0
    ),
    policy = unicycle_policy
)

function test_unicycle_controller(; plot=true)
    dynamics = unicycle_simple_dynamics()
    controller = unicycle_controller()
    task = unicycle_figure_eight_task()
    task_time = LearningWithSimpleModels.end_time(task)
    state0 = zeros(4)

    dt = 0.01
    ts = 0.0:dt:task_time
    state = state0
    states = zeros(length(state0), length(ts))
    setpoints = zeros(4, length(ts))
    us = zeros(2, length(ts))
    for (i,t) in enumerate(ts)
        setpoint = evaluate(task, t)
        u = LearningWithSimpleModels.next_command(controller, state, setpoint)
        state = LearningWithSimpleModels.f_simple(dynamics, t, dt, state, u)
        states[:,i] = state
        setpoints[:,i] = setpoint
        us[:,i] = u
    end
    
    if plot
        xs = states[1,:]
        ys = states[2,:]
        vs = states[3,:]
        ϕs = states[4,:]
        xdes = setpoints[1,:]
        ydes = setpoints[2,:]
        xdot_des = setpoints[3,:]
        ydot_des = setpoints[4,:]
        vdes = sqrt.(xdot_des.^2 .+ ydot_des.^2)
        as = us[1,:]
        ωs = us[2,:]

        fig = Figure(; resolution=(1000,800))

        ax_xy = Axis(fig[1:2,1:2], xlabel="x", ylabel="y")
        lines!(ax_xy, xs, ys, label ="actual")
        lines!(ax_xy, xdes, ydes, label ="desired")

        ax_x = Axis(fig[3,1], xlabel="t", ylabel="x")
        lines!(ax_x, ts, xs, label ="actual")
        lines!(ax_x, ts, xdes, label ="desired")

        ax_y = Axis(fig[3,2], xlabel="t", ylabel="y")
        lines!(ax_y, ts, ys, label ="actual")
        lines!(ax_y, ts, ydes, label ="desired")

        ax_v = Axis(fig[4,1:2], xlabel="t", ylabel="v")
        lines!(ax_v, ts, vs, label ="actual")
        lines!(ax_v, ts, vdes, label ="desired")

        ax_a = Axis(fig[5,1], xlabel="t", ylabel="a")
        lines!(ax_a, ts, as)

        ax_ω = Axis(fig[5,2], xlabel="t", ylabel="ω")
        lines!(ax_ω, ts, ωs)

        display(GLMakie.Screen(), fig)
    end
end
