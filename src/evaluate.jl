function evaluate_model(;
    actual_dynamics::Dynamics,
    controller::Controller, 
    cost::Cost,
    task::AbstractTask,
    algo::TrainingAlgorithm,
    training_params::TrainingParameters,
    sim_params::SimulationParameters,
    eval_params::EvaluationParameters,
    model = nothing
)
    if isnothing(model)
        model_filename = eval_params.name * "_train_model.bson"
        model_path = joinpath(eval_params.path, model_filename)
        Core.eval(Main, :(using Flux))
        @load model_path model
    end

    r = rollout_actual_dynamics(task, model, actual_dynamics, controller, cost, sim_params, eval_params)

    r_no_model = rollout_actual_dynamics(
        task, model, actual_dynamics, controller, cost, sim_params, eval_params
        ; use_model = false
    )

    xs_task, ys_task, _, _ = eval_all(task, r.ts)

    eval_data = EvaluationData(;
        r = r,
        r_no_model = r_no_model,
        xs_task = xs_task,
        ys_task = ys_task,
    )

    plot_evaluation(;
        eval_params = eval_params,
        eval_data = eval_data,
        algo = algo,
        training_params = training_params,
        sim_params = sim_params,
    )
    animate_evaluation(eval_params, eval_data)

    return eval_data
end

function evaluate_on_hardware(;
    task::AbstractTask,
    ctrl_params::ControllerParameters,
    algo::HardwareTrainingAlgorithm,
    sim_params::SimulationParameters,
    eval_params::EvaluationParameters,
    use_model::Bool = true
)
    model_filename = eval_params.name * "_train_model.bson"
    model_path = joinpath(eval_params.path, model_filename)
    Core.eval(Main, :(using Flux))
    @load model_path model

    task_time = end_time(task)
    n_segments = eval_params.n_task_executions * Integer(round(task_time/sim_params.model_dt))

    connections = open_connections()
    sleep(0.5)
    r = rollout_actual_dynamics(connections, task, model, algo, sim_params,
        ctrl_params, n_segments; use_model)
    close_connections(connections)

    suffix = use_model ? "_rollout_using_model.bson" : "_rollout_no_model.bson"
    filename = eval_params.name * suffix
    path = joinpath(eval_params.path, filename)
    if use_model
        @save path r
    else
        r_no_model = r
        @save path r_no_model
    end
end