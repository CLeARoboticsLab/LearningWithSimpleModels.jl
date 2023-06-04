struct DpOpenLoopControllerParameters <:ControllerParameters end

function dp_open_loop_policy(
    controller::Controller, 
    state::Vector{Float64}, 
    setpoints::Vector{Float64}, 
    gains_adjustment::Vector{Float64}
)
    return setpoints[1:2]
end

dp_controller_open_loop() = Controller(;
    params = DpOpenLoopControllerParameters(),
    policy = dp_open_loop_policy
)