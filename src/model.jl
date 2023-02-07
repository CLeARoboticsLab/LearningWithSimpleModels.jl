function make_model(layer_sizes::Vector{<:Integer})
    layers = []
    for i in 1:length(layer_sizes)-1
        push!(
            layers,
            Dense(
                layer_sizes[i] => layer_sizes[i+1], 
                tanh;
                init = zeros32
            )
        )
    end
    return Chain(layers...,)
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