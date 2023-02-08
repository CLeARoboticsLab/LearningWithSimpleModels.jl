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

    ts, xs, us, t0_segs, x0_segs, _ = rollout_actual_dynamics(
        task, model, actual_dynamics, controller, sim_params
    )

    xs_task, ys_task, _, _ = eval_all(task, ts)

    return EvaluationData(;
        ts = ts,
        xs = xs,
        us = us, 
        t0_segs = t0_segs,
        x0_segs = x0_segs,
        xs_task = xs_task,
        ys_task = ys_task
    )
end