function train(
    simple_dynamics::Dynamics,
    actual_dynamics::Dynamics
    ; controller::Controller,
    cost::Cost,
    task::AbstractTask,
    algo::TrainingAlgorithm,
    training_params::TrainingParameters,
    sim_params::SimulationParameters  
)
    p = ProgressMeter.Progress(training_params.iters)
    model = make_model(length(sim_params.x0), training_params.hidden_layer_sizes)
    
    if training_params.optim == gradient_descent
        optimizer = setup(Descent(training_params.learning_rate), model)
    else
        optimizer = setup(Adam(training_params.learning_rate), model)
    end

    losses = zeros(training_params.iters)
    rollouts = Vector{RolloutData}(undef,0)
    for i in 1:training_params.iters
        loss, rs = policy_update!(
            algo, task, model, optimizer, simple_dynamics, actual_dynamics,
            controller, cost, training_params, sim_params
        )
        ProgressMeter.next!(p, showvalues = [(:loss,loss)])
        losses[i] = loss
        append!(rollouts, rs)
    end
    
    save_model(training_params, model)
    plot_losses(training_params, losses)
    animate_training(training_params, rollouts, task)

    return model, losses
end

function train(
    simple_dynamics::Dynamics
    ; controller::Controller,
    ctrl_params::ControllerParameters,
    cost::Cost,
    task::AbstractTask,
    algo::TrainingAlgorithm,
    training_params::TrainingParameters,
    sim_params::SimulationParameters  
)
    write_params(algo, training_params, sim_params)
    p = ProgressMeter.Progress(training_params.iters)
    model = make_model(length(sim_params.x0), training_params.hidden_layer_sizes)

    if training_params.optim == gradient_descent
        optimizer = setup(Descent(training_params.learning_rate), model)
    else
        optimizer = setup(Adam(training_params.learning_rate), model)
    end

    losses = Vector{Float64}(undef,0)
    rollouts = Vector{RolloutData}(undef,0)
    connections = open_connections()
    sleep(0.5)
    for i in 1:training_params.iters
        loss, r = policy_update!(algo, connections, task, model, optimizer, simple_dynamics,
                                 controller, ctrl_params, cost, sim_params)
        ProgressMeter.next!(p, showvalues = [(:loss,loss)])
        push!(losses, loss)
        push!(rollouts, r)
        plot_rollout(r, task, loss, losses)
    end
    close_connections(connections)

    save_model(training_params, model)
    plot_losses(training_params, losses)
    save_all_data(training_params, 
        TrainingData(;
            losses = losses,
            rollouts = rollouts
        )
    )
    # TODO need to figure out how to animate rollouts of varying lengths
    # (probably need to make arrays of Point2f)
    # animate_training(training_params, rollouts, task)
end

function write_params(    
    algo::TrainingAlgorithm,
    training_params::TrainingParameters,
    sim_params::SimulationParameters
)
    if isnothing(training_params.save_path)
        return
    end

    filename = training_params.name * "_params.txt"
    path = joinpath(training_params.save_path, filename)
    open(path,"w") do io
        println(io, algo)
        println(io, training_params)
        println(io, sim_params)
     end
end

function save_model(training_params::TrainingParameters, model::Chain)
    if isnothing(training_params.save_path) || !training_params.save_model
        return
    end

    filename = training_params.name * "_train_model.bson"
    path = joinpath(training_params.save_path, filename)
    @save path model
end

function save_all_data(training_params::TrainingParameters, data::TrainingData)
    if isnothing(training_params.save_path) || !training_params.save_all_data
        return
    end

    filename = training_params.name * "_training_data.bson"
    path = joinpath(training_params.save_path, filename)
    @save path data
end