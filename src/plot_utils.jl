function plot_losses(losses)
    fig = Figure()
    ax = Axis(fig[1,1], xlabel="Iteration", ylabel="Loss")
    lines!(ax, losses)
    display(GLMakie.Screen(), fig)
    return fig
end

function plot_losses(training_params::TrainingParameters, losses)
    fig = plot_losses(losses)
    path = training_params.plot_save_path
    if !isnothing(path)
        save(path, fig)
    end
    display(GLMakie.Screen(), fig)
end

function plot_evaluation(
    eval_data::EvaluationData;
    training_params::Union{TrainingParameters, Nothing} = nothing,
    sim_params::Union{SimulationParameters, Nothing} = nothing,
    save_path = nothing
)
    fig = Figure()
    ax = Axis(fig[1,1:2], xlabel="x", ylabel="y")
    lines!(ax, eval_data.r.xs[1,:], eval_data.r.xs[2,:], label="Trajectory")
    lines!(ax, eval_data.r_no_model.xs[1,:], eval_data.r_no_model.xs[2,:], label="Trajectory w/o model")
    lines!(ax, eval_data.xs_task, eval_data.ys_task, label="Task", linestyle=:dash, color=:black)
    Legend(fig[1,3], ax)
    
    if !isnothing(training_params)
        Label(fig[2,1], string(training_params))
    end

    if !isnothing(sim_params)
        Label(fig[2,2], string(sim_params))
    end

    save(save_path, fig)
    display(GLMakie.Screen(), fig)
end

function plot_task(task::Spline, sim_params::SimulationParameters)
    task_time, _, _ = properties(task, sim_params)
    ts = 0.0:sim_params.dt:task_time
    xs_task, ys_task, _, _ = eval_all(task, ts)
    
    fig = Figure()
    ax = Axis(fig[1,1], xlabel="x", ylabel="y")
    lines!(ax, xs_task, ys_task, label="Task")
    display(GLMakie.Screen(), fig)
end