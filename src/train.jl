function train(;
    simple_dynamics::SimpleDynamics,
    actual_dynamics::ActualDynamics,
    controller::Controller,
    cost::Cost,
    task::Spline,
    n_states::Integer,
    hidden_layer_sizes::Vector{<:Integer},
    learning_rate = 1e-3,
    iters = 50    
)
    p = ProgressMeter.Progress(iters)
    model = make_model(n_states, hidden_layer_sizes)
    optimizer = setup(Adam(learning_rate), model)
    
    for i in 1:iters
        policy_update!()
        ProgressMeter.next!(p)
    end

    #TODO this is temp code
    println(f_simple(simple_dynamics,1.0,ones(4),ones(2)))
    println(f_actual(actual_dynamics,1.0,ones(4),ones(2)))
    println(stage_cost(cost,ones(4),ones(2)))
end

function policy_update!()
    
end