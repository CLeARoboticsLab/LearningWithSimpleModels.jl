abstract type ControllerParameters end

Base.@kwdef struct Controller
    params::ControllerParameters
    policy::Function
end

next_command(controller::Controller, x, x_des) = controller.policy(controller, x, x_des)
