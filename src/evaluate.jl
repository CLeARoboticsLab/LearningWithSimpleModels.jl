function evaluate_model(;
    actual_dynamics::Dynamics,
    controller::Controller, 
    task::Spline, 
    sim_params::SimulationParameters,
    model = nothing, 
    load_path = nothing
)
    if !isnothing(load_path)
        Core.eval(Main, :(using Flux))
        @load load_path model
    end

    r = rollout_actual_dynamics(task, model, actual_dynamics, controller, sim_params)

    r_no_model = rollout_actual_dynamics(
        task, model, actual_dynamics, controller, sim_params
        ; use_model = false
    )

    xs_task, ys_task, _, _ = eval_all(task, r.ts)

    return EvaluationData(;
        r = r,
        r_no_model = r_no_model,
        xs_task = xs_task,
        ys_task = ys_task,
    )
end
