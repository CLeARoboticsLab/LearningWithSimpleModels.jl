function make_model(layer_sizes::Vector{<:Integer})
    layers = []
    for i in 1:length(layer_sizes)-1
        layer_init = i != length(layer_sizes)-1 ? glorot_uniform : zeros32
        push!(
            layers,
            Dense(
                layer_sizes[i] => layer_sizes[i+1],
                tanh;
                init = layer_init
            )
        )
    end
    return Chain(layers...,) |> f64
end

function make_model(n_states::Integer, hidden_layer_sizes::Vector{<:Integer})
    layer_sizes = vcat(n_states+2, hidden_layer_sizes, 4+6)
    return make_model(layer_sizes)
end

function call_model(
    sim_params::SimulationParameters,
    setpoint::Vector{Float64}, 
    model::Chain, 
    t::Float64, 
    x::Vector{Float64},
    task_time::Float64
)
    t_transformed = [cos(2*π*t/task_time), sin(2*π*t/task_time)]
    x_transformed = [
        cos(x[1]),#x[1]/sim_params.model_scale[1],
        sin(x[1]),#x[2]/sim_params.model_scale[2],
        cos(x[2]),#x[3]/sim_params.model_scale[3],#x[3]*cos(x[4]),
        sin(x[2])#x[4]/sim_params.model_scale[4]#x[3]*sin(x[4])
    ] #TODO make this more robust to arbitrary state definitions
    model_out = model(vcat(x_transformed, t_transformed)) .* sim_params.model_scale
    setpoint_correction = model_out[1:4]
    gains_adjustment = model_out[5:10]
    return setpoint + setpoint_correction, gains_adjustment
end