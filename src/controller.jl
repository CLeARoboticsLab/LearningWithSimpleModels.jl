next_command(
    controller::Controller, 
    x::Vector{Float64}, 
    x_des::Vector{Float64},
    gains_adjustment::Vector{Float64}
) = controller.policy(controller, x, x_des, gains_adjustment)

next_command(
    controller::Controller, 
    x::Vector{Float64}, 
    x_des::Vector{Float64}
) = controller.policy(controller, x, x_des, zeros(4))