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

function make_model(n_inputs::Integer, hidden_layer_sizes::Vector{<:Integer})
    layer_sizes = vcat(n_inputs, hidden_layer_sizes, 4+6)
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
    inputs = sim_params.model_input_function(t, task_time, x)
    model_out = model(inputs) .* sim_params.model_scale
    setpoint_correction = model_out[1:4]
    gains_adjustment = model_out[5:10]
    # println("inputs: ", inputs, " outputs: ", model_out)
    return setpoint + setpoint_correction, gains_adjustment
end