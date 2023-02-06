Base.@kwdef struct UnicycleControllerParameters <: ControllerParameters
    kx::Float64
    ky::Float64
    kv::Float64
    kϕ::Float64
end

function unicycle_policy(controller::Controller, state, setpoints)
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
    v_des = sqrt(xdot_des^2 + ydot_des^2)

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


function test_controller()
    dynamics = unicycle_simple_dynamics()
    controller = unicycle_controller()
    time = 8.0
    task = LearningWithSimpleModels.figure_eight(;xdot_f=0.0, ydot_f=0.0, time=time)
    state0 = zeros(4)

    ts = 0.0:0.01:time
    state = state0
    states = zeros(length(state0), length(ts))
    setpoints = zeros(4, length(ts))
    for (i,t) in enumerate(ts)
        setpoint = collect(evaluate(task, t))
        u = LearningWithSimpleModels.next_command(controller, state, setpoint)
        state = LearningWithSimpleModels.f_simple(dynamics, t, state, u)
        states[:,i] = state
        setpoints[:,i] = setpoint
    end
    
    xs = states[1,:]
    ys = states[2,:]
    vs = states[3,:]
    ϕs = states[4,:]
    xdes = setpoints[1,:]
    ydes = setpoints[2,:]
    xdot_des = setpoints[3,:]
    ydot_des = setpoints[4,:]
    vdes = sqrt.(xdot_des.^2 .+ ydot_des.^2)

    fig = Figure(; resolution=(1000,800))
    lines(fig[1:2,1:2], xs, ys, label ="actual"#=, xlabel="x", ylabel="y"=#)
    lines!(fig[1:2,1:2], xdes, ydes, label ="desired"#=, xlabel="x", ylabel="y"=#)
    lines(fig[3,1], ts, xs, label ="actual"#=, xlabel="t", ylabel="x"=#)
    lines!(fig[3,1], ts, xdes, label ="desired"#=, xlabel="t", ylabel="x"=#)
    lines(fig[3,2], ts, ys, label ="actual"#=, xlabel="t", ylabel="y"=#)
    lines!(fig[3,2], ts, ydes, label ="desired"#=, xlabel="t", ylabel="y"=#)
    lines(fig[4,1], ts, vs, label ="actual"#=, xlabel="t", ylabel="v"=#)
    lines!(fig[4,1], ts, vdes, label ="desired"#=, xlabel="t", ylabel="v"=#)
    # lines(fig[4,2], ts, ϕs, label ="actual"#=, xlabel="t", ylabel="ϕ"=#)
    # lines!(fig[4,2], ts, ϕdes, label ="desired"#=, xlabel="t", ylabel="ϕ"=#)
    display(GLMakie.Screen(), fig)

    #TODO: add types to func sigs for f and policy funcs?
end
