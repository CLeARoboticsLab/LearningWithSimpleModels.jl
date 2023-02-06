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
