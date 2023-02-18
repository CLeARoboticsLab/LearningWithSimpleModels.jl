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
    algo::Union{TrainingAlgorithm, Nothing} = nothing,
    training_params::Union{TrainingParameters, Nothing} = nothing,
    sim_params::Union{SimulationParameters, Nothing} = nothing,
    save_path = nothing
)
    fig = Figure(resolution=(850,600))
    ax = Axis(fig[1,1:2], xlabel="x", ylabel="y")
    lines!(ax, eval_data.r.xs[1,:], eval_data.r.xs[2,:], label="Trajectory")
    lines!(ax, eval_data.r_no_model.xs[1,:], eval_data.r_no_model.xs[2,:], label="Trajectory w/o model")
    lines!(ax, eval_data.xs_task, eval_data.ys_task, label="Task", linestyle=:dash, color=:black)
    Legend(fig[1,3], ax)
    
    label = string(algo) * "
    
    Loss: $(round(eval_data.r.loss; digits=3))"
    if !isnothing(algo)
        Label(fig[2,1], label)
    end

    if !isnothing(training_params)
        Label(fig[2,2], string(training_params))
    end

    if !isnothing(sim_params)
        Label(fig[2,3], string(sim_params))
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

#TODO: clean this up
function animate_training(rollouts::Vector{RolloutData}, task::Spline)
    xs_task, ys_task, _, _ = eval_all(task, rollouts[1].ts)
    len = length(xs_task)

    iter = Observable(0)
    cost = Observable(0.0)

    fig = Figure()
    ax = Axis(fig[1,1], xlabel="x", ylabel="y", title=@lift("Iteration: $($iter); Cost: $(round($cost))"))
    l = lines!(ax, xs_task, ys_task, label="Task", linestyle=:dash, color=:black)
    l = scatter!(ax, rollouts[1].xs[1,:], rollouts[1].xs[2,:], color = range(0, 1, length=len), colormap=:thermal, markersize=10)
    sc = scatter!(ax, rollouts[1].xs[1,1], rollouts[1].xs[2,1], color=:red, markersize=20)

    record(fig, ".data/training_animation.mp4", 1:length(rollouts); framerate = 1) do i
        l[1] = rollouts[i].xs[1,:]
        l[2] = rollouts[i].xs[2,:]
        sc[1] = rollouts[i].xs[1,1]
        sc[2] = rollouts[i].xs[2,1]
        iter[] = i
        cost[] = rollouts[i].loss
    end
end