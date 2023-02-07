Base.@kwdef struct TrainingParameters
    n_states::Integer
    dt::Float64 = 0.01
    hidden_layer_sizes::Vector{<:Integer} = [64, 64]
    learning_rate::Float64 = 1e-3
    iters::Integer = 50
end

function train(;
    simple_dynamics::SimpleDynamics,
    actual_dynamics::ActualDynamics,
    controller::Controller,
    cost::Cost,
    task::Spline,
    params::TrainingParameters  
)
    p = ProgressMeter.Progress(params.iters)
    model = make_model(params.n_states, params.hidden_layer_sizes)
    optimizer = setup(Adam(params.learning_rate), model)
    
    for i in 1:params.iters
        policy_update!()
        ProgressMeter.next!(p)
    end

    #TODO this is temp code
    println(f_simple(simple_dynamics,1.0,params.dt,ones(4),ones(2)))
    println(f_actual(actual_dynamics,1.0,params.dt,ones(4),ones(2)))
    println(stage_cost(cost,ones(4),ones(2)))
end

function policy_update!()
    
end