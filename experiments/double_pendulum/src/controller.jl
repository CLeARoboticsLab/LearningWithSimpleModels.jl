Base.@kwdef struct DpControllerParameters <: ControllerParameters # TODO
    gains::Vector{Float64}
    gains_names::Vector{String} = ["Kx","Ky","Kvx","Kϕ","Kvy","Kω"]
end

dp_controller_parameters() = DpControllerParameters(; # TODO
    gains = [
        0.50, # kx
        0.50, # ky
        0.0, # kvx
        2.50, # kϕ
        0.0, # kvy
        0.00, # kω
    ]
)

function dp_policy(
    controller::Controller, 
    state::Vector{Float64}, 
    setpoints::Vector{Float64}, 
    gains_adjustment::Vector{Float64}
)
    return [0.0, 0.0] # TODO
end

dp_controller() = Controller(;
    params = dp_controller_parameters(),
    policy = dp_policy
)