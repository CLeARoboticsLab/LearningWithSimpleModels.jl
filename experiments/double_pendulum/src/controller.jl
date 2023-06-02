Base.@kwdef struct DpControllerParameters <: ControllerParameters
    gains::Matrix{Float64}
    gains_names::Vector{String} = ["K11","K22","K13","K24","null","null"]
end

dp_controller_parameters() = DpControllerParameters(;
    gains = controller_gains()
)

function controller_gains()
    # LQR cost weighting matrices
    Q = diagm([10, 10, 1, 1])
    R = diagm([1.0, 1.0])
    p = dp_simple_dynamics_params()
    
    # Lienarize system about straight up equilibrium point
    x_nom = [π, 0.0, 0.0, 0.0]
    u_nom = [0.0, 0.0]
    fx(x) = [ x[3:4] ; inv(H(x[1:2],p))*( B(x[1:2],p)*u_nom - C(x[1:2],x[3:4],p)*x[3:4] - G(x[1:2],p) )]
    fu(u) = [ x_nom[3:4] ; inv(H(x_nom[1:2],p))*( B(x_nom[1:2],p)*u - C(x_nom[1:2],x_nom[3:4],p)*x_nom[3:4] - G(x_nom[1:2],p) )]
    A_mat = jacobian(fx, x_nom)
    B_mat = jacobian(fu, u_nom)

    # Compute LQR gains
    Π, _, _, _ = arec(A_mat, B_mat, R, Q, zeros(size(B_mat)))
    K_LQR = inv(R)*B_mat'*Π

    return K_LQR
end

function dp_policy(
    controller::Controller, 
    state::Vector{Float64}, 
    setpoints::Vector{Float64}, 
    gains_adjustment::Vector{Float64}
)
    K11 = gains_adjustment[1]
    K22 = gains_adjustment[2]
    K13 = gains_adjustment[3]
    K24 = gains_adjustment[4]
    ΔK = [
        K11 0   K13 0
        0   K22 0   K24
    ]
    K = controller.params.gains + ΔK
    u = -K*(state - setpoints[1:4])
    return u
end

dp_controller() = Controller(;
    params = dp_controller_parameters(),
    policy = dp_policy
)