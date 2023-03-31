Base.@kwdef struct QuadrupedControllerParameters <: ControllerParameters
    kx::Float64
    ky::Float64
    kv::Float64
    kϕ::Float64
    ka::Float64
    kω::Float64
    limit::Bool
    v_limit::Float64
    ω_limit::Float64
end

quadruped_controller_parameters() = QuadrupedControllerParameters(;
    kx = 0.2,
    ky = 0.2,
    kv = 0.00,
    kϕ = 1.00,
    ka = 0.00,
    kω = 0.00,
    limit = true,
    v_limit = 1.0,
    ω_limit = 1.0
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
    ω_des = -xddot_des/v_des_og*sin(ϕ_des_og) + yddot_des/v_des_og*cos(ϕ_des_og)

    ω = (controller.params.kω + Δkω)*ω_des + (controller.params.kϕ + Δkϕ)*(ϕ_des - ϕ)
    
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