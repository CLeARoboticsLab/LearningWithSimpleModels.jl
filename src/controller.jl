next_command(
    controller::Controller, 
    x::Vector{Float64}, 
    x_des::Vector{Float64}
) = controller.policy(controller, x, x_des)
