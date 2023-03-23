function plot_losses(training_params::TrainingParameters, losses)
    fig = Figure()
    ax = Axis(fig[1,1], xlabel="Iteration", ylabel="Loss")
    lines!(ax, losses)

    if !isnothing(training_params.save_path) && training_params.save_plot
        filename = training_params.name * "_train_loss.png"
        path = joinpath(training_params.save_path, filename)
        save(path, fig)
    end

    display(GLMakie.Screen(), fig)
end

function plot_evaluation(;
    eval_params::EvaluationParameters,
    eval_data::EvaluationData,
    algo::Union{TrainingAlgorithm, Nothing} = nothing,
    training_params::Union{TrainingParameters, Nothing} = nothing,
    sim_params::Union{SimulationParameters, Nothing} = nothing,
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

    #TODO clean this up (control inputs plot) 
    fig2 = Figure(resolution=(600,600))
    ax1 = Axis(fig2[1,1], xlabel="t", ylabel="a")
    ax2 = Axis(fig2[2,1], xlabel="t", ylabel="ω")
    lines!(ax1, eval_data.r.ts, eval_data.r.us[1,:], label="accel")
    lines!(ax2, eval_data.r.ts, eval_data.r.us[2,:], label="turn rate")
    
    fig3 = Figure(resolution=(600,600))
    ax3_1 = Axis(fig3[1,1], xlabel="t", ylabel="Δkx")
    ax3_2 = Axis(fig3[2,1], xlabel="t", ylabel="Δky")
    ax3_3 = Axis(fig3[3,1], xlabel="t", ylabel="Δkv")
    ax3_4 = Axis(fig3[4,1], xlabel="t", ylabel="Δkϕ")
    lines!(ax3_1, eval_data.r.t0_segs, eval_data.r.gain_adjs[1,:], label="Δkx")
    lines!(ax3_2, eval_data.r.t0_segs, eval_data.r.gain_adjs[2,:], label="Δky")
    lines!(ax3_3, eval_data.r.t0_segs, eval_data.r.gain_adjs[3,:], label="Δkv")
    lines!(ax3_4, eval_data.r.t0_segs, eval_data.r.gain_adjs[4,:], label="Δkϕ")

    if !isnothing(eval_params.path) && eval_params.save_plot
        eval_plot_filename = eval_params.name * "_eval_plot.png"
        eval_plot_path = joinpath(eval_params.path, eval_plot_filename)
        save(eval_plot_path, fig)
    
        inputs_plot_filename = eval_params.name * "_eval_control_inputs.png"
        inputs_plot_path = joinpath(eval_params.path, inputs_plot_filename)
        save(inputs_plot_path, fig2)

        gains_plot_filename = eval_params.name * "_eval_gains.png"
        gains_plot_path = joinpath(eval_params.path, gains_plot_filename)
        save(gains_plot_path, fig3)
    end

    display(GLMakie.Screen(), fig)
    display(GLMakie.Screen(), fig2)
    display(GLMakie.Screen(), fig3)
end

function plot_task(task::Spline, sim_params::SimulationParameters)
    task_time, _ = properties(task, sim_params)
    ts = 0.0:sim_params.dt:task_time
    xs_task, ys_task, _, _ = eval_all(task, ts)
    
    fig = Figure()
    ax = Axis(fig[1,1], xlabel="x", ylabel="y")
    lines!(ax, xs_task, ys_task, label="Task")
    display(GLMakie.Screen(), fig)
end

#TODO: separate rollouts by policy update
function animate_training(training_params::TrainingParameters, rollouts::Vector{RolloutData}, task::Spline)
    if isnothing(training_params.save_path) || !training_params.save_animation
        return
    end

    filename = training_params.name * "_train_animation.mp4"
    path = joinpath(training_params.save_path, filename)
    
    xs_task, ys_task, _, _ = eval_all(task, rollouts[1].ts)
    len = length(xs_task)

    iter = Observable(0)
    cost = Observable(0.0)

    fig = Figure()
    ax = Axis(fig[1,1], xlabel="x", ylabel="y", title=@lift("Rollout: $($iter); Cost: $(round($cost))"))
    l = lines!(ax, xs_task, ys_task, label="Task", linestyle=:dash, color=:black)
    l = scatter!(ax, rollouts[1].xs[1,:], rollouts[1].xs[2,:], color = range(0, 1, length=len), colormap=:thermal, markersize=5)
    sc = scatter!(ax, rollouts[1].xs[1,1], rollouts[1].xs[2,1], color=:red, markersize=20)

    record(fig, path, 1:length(rollouts); framerate = 2) do i
        l[1] = rollouts[i].xs[1,:]
        l[2] = rollouts[i].xs[2,:]
        sc[1] = rollouts[i].xs[1,1]
        sc[2] = rollouts[i].xs[2,1]
        iter[] = i
        cost[] = rollouts[i].loss
    end
end

function animate_evaluation(eval_params::EvaluationParameters, eval_data::EvaluationData)
    if isnothing(eval_params.path) || !eval_params.save_animation
        return
    end
    
    filename = eval_params.name * "_eval_animation.mp4"
    path = joinpath(eval_params.path, filename)

    traj_points = Observable(Point2f[(eval_data.r.xs[1,1], eval_data.r.xs[2,1])])
    # ctrl_set_points = Observable(Point2f[(eval_data.r.ctrl_setpoints[1,1], eval_data.r.ctrl_setpoints[2,1])])
    set_points = Observable(Point2f[(eval_data.r.setpoints[1,1], eval_data.r.setpoints[2,1])])
    task_points = Observable(Point2f[(eval_data.xs_task[1], eval_data.ys_task[1])])
    T = length(eval_data.r.ts)

    fig = Figure(resolution=(850,600))
    ax = Axis(fig[1,1], xlabel="x", ylabel="y")
    limits!(ax, -8, 8, -5, 5)
    lines!(ax, task_points, label="Task", linestyle=:dash, color=:black)
    # lines!(ax, ctrl_set_points, label="Controller setpoints", linestyle=:dash, color=:red)

    scatter!(ax, traj_points, color = range(0.5,1.0, length=T), colormap=:thermal, markersize=7)
    scatter!(ax, set_points, color = range(0.5,1.0, length=T), colormap=:thermal, markersize=20)

    j = 1
    record(fig, path, 1:T; framterate = 100) do i
        traj_points[] = push!(traj_points[], Point2f(eval_data.r.xs[1,i], eval_data.r.xs[2,i]))
        # ctrl_set_points[] = push!(ctrl_set_points[], Point2f(eval_data.r.ctrl_setpoints[1,i], eval_data.r.ctrl_setpoints[2,i]))
        if j <= length(eval_data.r.idx_segs) && i >= eval_data.r.idx_segs[j]
            set_points[] = push!(set_points[], Point2f(eval_data.r.setpoints[1,j], eval_data.r.setpoints[2,j]))
            j += 1
        end
        task_points[] = push!(task_points[], Point2f(eval_data.xs_task[i],eval_data.ys_task[i]))
    end
end