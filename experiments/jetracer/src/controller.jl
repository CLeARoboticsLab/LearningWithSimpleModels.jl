Base.@kwdef struct JetracerControllerParameters <: ControllerParameters
    gains::Vector{Float64}
    gains_names::Vector{String} = ["Kx","Ky","Kv","Kϕ","Ka","Kω"]
    limit::Bool
    a_limit::Float64
    ω_limit::Float64
end

jetracer_controller_parameters() = JetracerControllerParameters(;
    gains = [
        0.45, # kx
        0.45, # ky
        0.35, # kv
        0.35, # kϕ
        0.00, # ka
        0.00, # kω
    ],
    limit = true,
    a_limit = 0.75,
    ω_limit = 1.0
)

function jetracer_policy(
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
    kv = controller.params.gains[3]
    kϕ = controller.params.gains[4]
    ka = controller.params.gains[5]
    kω = controller.params.gains[6]

    Δkx = gains_adjustment[1]
    Δky = gains_adjustment[2]
    Δkv = gains_adjustment[3]
    Δkϕ = gains_adjustment[4]
    Δka = gains_adjustment[5]
    Δkω = gains_adjustment[6]    

    xdot_tilde_des = xdot_des + (kx + Δkx)*(x_des - x)
    ydot_tilde_des = ydot_des + (ky + Δky)*(y_des - y)
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

    a = (ka + Δka)*a_des + (kv + Δkv)*(v_des - v)
    ω = (kω + Δkω)*ω_des + (kϕ + Δkϕ)*(ϕ_des - ϕ)
    
    if controller.params.limit
        a_lim = controller.params.a_limit
        ω_lim = controller.params.ω_limit
        return [clamp(a,-a_lim,a_lim), clamp(ω,-ω_lim,ω_lim)]
    else
        return [a, ω]
    end
end

jetracer_controller() = Controller(;
    params = jetracer_controller_parameters(),
    policy = jetracer_policy
)