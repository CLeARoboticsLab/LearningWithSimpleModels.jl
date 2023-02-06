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

    xdot_tilde = xdot_des + controller.params.kx*(x_des - x)
    ydot_tilde = ydot_des + controller.params.ky*(y_des - y)
    v_des = sqrt(xdot_des^2 + ydot_des^2)
    ϕ_des = atan(ydot_tilde, xdot_tilde)

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
    controller = unicycle_controller()
    LearningWithSimpleModels.next_command(controller, ones(4), .5*ones(4))
end
