Base.@kwdef struct QuadrupedControllerParameters <: ControllerParameters
    gains::Vector{Float64}
    gains_names::Vector{String} = ["kx","ky","kvx","kϕ","kvy","kω"]
    limit::Bool
    v_limit::Float64
    ω_limit::Float64
end

quadruped_controller_parameters() = QuadrupedControllerParameters(;
    gains = [
        0.20, # kx
        0.20, # ky
        0.95, # kvx
        2.50, # kϕ
        0.95, # kvy
        0.00, # kω
    ],
    limit = true,
    v_limit = 1.0,
    ω_limit = 3.0
)

function quadruped_policy(
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

    kx = controller.params.gains[1]
    ky = controller.params.gains[2]
    kvx = controller.params.gains[3]
    kϕ = controller.params.gains[4]
    kvy = controller.params.gains[5]
    kω = controller.params.gains[6]

    Δkx = gains_adjustment[1]
    Δky = gains_adjustment[2]
    Δkvx = gains_adjustment[3]
    Δkϕ = gains_adjustment[4]
    Δkvy = gains_adjustment[5]
    Δkω = gains_adjustment[6]    

    xdot_tilde_des = (kvx + Δkvx)*xdot_des + (kx + Δkx)*(x_des - x)
    ydot_tilde_des = (kvy + Δkvy)*ydot_des + (ky + Δky)*(y_des - y)
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
    ω_des = -xddot_des/v_des_og*sin(ϕ_des_og) + yddot_des/v_des_og*cos(ϕ_des_og)

    ω = (kω + Δkω)*ω_des + (kϕ + Δkϕ)*(ϕ_des - ϕ)
    
    if controller.params.limit
        v_lim = controller.params.v_limit
        ω_lim = controller.params.ω_limit
        return [clamp(v_des,-v_lim,v_lim), clamp(ω,-ω_lim,ω_lim)]
    else
        return [v_des, ω]
    end
end

quadruped_controller() = Controller(;
    params = quadruped_controller_parameters(),
    policy = quadruped_policy
)