struct DpOpenLoopControllerParameters <:ControllerParameters end

function dp_open_loop_policy(
    controller::Controller, 
    state::Vector{Float64}, 
    setpoints::Vector{Float64}, 
    gains_adjustment::Vector{Float64}
)
    start_state = start_state_open_loop()
    u_nom = G(start_state[1:2], dp_simple_dynamics_params_open_loop())
    return u_nom + setpoints[1:2]
end

dp_controller_open_loop() = Controller(;
    params = DpOpenLoopControllerParameters(),
    policy = dp_open_loop_policy
)