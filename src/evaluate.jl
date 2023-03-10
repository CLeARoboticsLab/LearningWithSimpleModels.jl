function evaluate_model(;
    actual_dynamics::Dynamics,
    controller::Controller, 
    cost::Cost,
    task::Spline,
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
