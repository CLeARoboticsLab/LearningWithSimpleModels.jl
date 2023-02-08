function make_model(layer_sizes::Vector{<:Integer})
    layers = []
    for i in 1:length(layer_sizes)-1
        push!(
            layers,
            Dense(
                layer_sizes[i] => layer_sizes[i+1], 
                tanh #TODO need to match initilization?
            )
        )
    end
    return Chain(layers...,)
end

function make_model(n_states::Integer, hidden_layer_sizes::Vector{<:Integer})
    layer_sizes = vcat(n_states+2, hidden_layer_sizes, 4)
    return make_model(layer_sizes)
end

function new_setpoint_from_model(
    setpoint::Vector{Float64}, 
    model::Chain, 
    t::Float64, 
    x::Vector{Float64},
    task_time::Float64
)
    t_transformed = [cos(2*π*t/task_time), sin(2*π*t/task_time)]
    setpoint_correction = model(vcat(x, t_transformed))
    return setpoint + setpoint_correction
end