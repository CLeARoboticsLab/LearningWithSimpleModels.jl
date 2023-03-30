Base.@kwdef struct UnicycleControllerParameters <: ControllerParameters
    kx::Float64
    ky::Float64
    kv::Float64
    kϕ::Float64
    ka::Float64
    kω::Float64
    limit::Bool
    a_limit::Float64
    ω_limit::Float64
end

function unicycle_policy(controller::Controller, state::Vector{Float64}, setpoints::Vector{Float64})
    return unicycle_policy(controller, state, setpoints, zeros(4))
end

function unicycle_policy(
    controller::Controller, 
    state::Vector{Float64}, 
    setpoints::Vector{Float64}, 
    gains_adjustment::Vector{Float64}
)
    x = state[1]
    y = state[2]
    v = state[3]
    ϕ = state[4]

    x_des = setpoints[1]
    y_des = setpoints[2]
    xdot_des = setpoints[3]
    ydot_des = setpoints[4]
    xddot_des = setpoints[5]
    yddot_des = setpoints[6]

    Δkx = gains_adjustment[1]
    Δky = gains_adjustment[2]
    Δkv = gains_adjustment[3]
    Δkϕ = gains_adjustment[4]
    Δka = gains_adjustment[5]
    Δkω = gains_adjustment[6]    

    xdot_tilde_des = xdot_des + (controller.params.kx + Δkx)*(x_des - x)
    ydot_tilde_des = ydot_des + (controller.params.ky + Δky)*(y_des - y)
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
    elseif abs(ϕ_des - ϕ) > abs(ϕ_des + 2*π - ϕ)
        ϕ_des += 2*π
    end

    v_des_og = sqrt(xdot_des^2 + ydot_des^2)
    ϕ_des_og = atan(ydot_des, xdot_des)

    a_des = cos(ϕ_des_og)*xddot_des + sin(ϕ_des_og)*yddot_des
    ω_des = -xddot_des/v_des_og*sin(ϕ_des_og) + yddot_des/v_des_og*cos(ϕ_des_og)

    a = (controller.params.ka + Δka)*a_des + (controller.params.kv + Δkv)*(v_des - v)
    ω = (controller.params.kω + Δkω)*ω_des + (controller.params.kϕ + Δkϕ)*(ϕ_des - ϕ)
    
    if controller.params.limit
        a_lim = controller.params.a_limit
        ω_lim = controller.params.ω_limit
        return [clamp(a,-a_lim,a_lim), clamp(ω,-ω_lim,ω_lim)]
    else
        return [a, ω]
    end
end

unicycle_controller() = Controller(;
    params = UnicycleControllerParameters(;
        kx = 0.45,
        ky = 0.45,
        kv = 0.35,
        kϕ = 0.35,
        ka = 0.00,
        kω = 0.00,
        limit = true,
        a_limit = 0.75,
        ω_limit = 1.0
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
    setpoints = zeros(6, length(ts))
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

        display(fig)
    end
end
