function train(
    simple_sys::SimpleDynamics,
    actual_sys::ActualDynamics,
    cost::Cost    
) #TODO come up w/ good fn signature

    #TODO this is temp code
    println(f_simple(simple_sys,1.0,ones(4),ones(2)))
    println(f_actual(actual_sys,1.0,ones(4),ones(2)))
    println(stage_cost(cost,ones(4),ones(2)))
end
