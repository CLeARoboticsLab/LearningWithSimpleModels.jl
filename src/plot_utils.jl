function plot_losses(training_params::TrainingParameters, losses)
    fig = Figure()
    ax = Axis(fig[1,1], xlabel="Iteration", ylabel="Loss")
    lines!(ax, losses)

    if !isnothing(training_params.save_path) && training_params.save_plot
        filename = training_params.name * "_train_loss.png"
        path = joinpath(training_params.save_path, filename)
        save(path, fig)
    end

    display(fig)
end

function plot_rollout(
    r::RolloutData,
    task::AbstractTask,
    loss::Float64,
    losses::Vector{Float64}  
)
    xs_task, ys_task, _, _ = eval_all(task, r.ts .+ r.task_t0)
    T = length(r.ts)
    segs = length(r.t0_segs)

    fig = Figure(resolution=(1400,800))

    # trajectory
    ax = Axis(fig[1:3,1:3], title="Loss: $(loss)", xlabel="x", ylabel="y")
    scatter!(ax, r.xs[1,:], r.xs[2,:], color = range(0.5,1.0, length=T), colormap=:thermal, markersize=7)
    lines!(ax, xs_task, ys_task, label="Task", linestyle=:dash, color=:black, linewidth=2.0)
    scatter!(ax, r.setpoints[1,:], r.setpoints[2,:], marker=:xcross, color=range(0.5,1.0, length=segs), colormap=:thermal, markersize=16)

    # control inputs
    ax1 = Axis(fig[4,1:3], xlabel="t", ylabel="a")
    ax2 = Axis(fig[5,1:3], xlabel="t", ylabel="ω")
    lines!(ax1, r.ts, r.us[1,:], label="accel")
    lines!(ax2, r.ts, r.us[2,:], label="turn rate")

    # gains
    ax3_1 = Axis(fig[1,4], xlabel="t", ylabel="Δkx")
    ax3_2 = Axis(fig[2,4], xlabel="t", ylabel="Δky")
    ax3_3 = Axis(fig[3,4], xlabel="t", ylabel="Δkv")
    ax3_4 = Axis(fig[4,4], xlabel="t", ylabel="Δkϕ")
    ax3_5 = Axis(fig[5,4], xlabel="t", ylabel="Δka")
    lines!(ax3_1, r.t0_segs .- r.task_t0, r.gain_adjs[1,:], label="Δkx")
    lines!(ax3_2, r.t0_segs .- r.task_t0, r.gain_adjs[2,:], label="Δky")
    lines!(ax3_3, r.t0_segs .- r.task_t0, r.gain_adjs[3,:], label="Δkv")
    lines!(ax3_4, r.t0_segs .- r.task_t0, r.gain_adjs[4,:], label="Δkϕ")
    lines!(ax3_5, r.t0_segs .- r.task_t0, r.gain_adjs[5,:], label="Δka")

    # control setpoints
    # vs = [sqrt(r.ctrl_setpoints[3,i]^2 + r.ctrl_setpoints[4,i]^2) for i in 1:length(r.ts)]
    ax4_1 = Axis(fig[1,5], xlabel="t", ylabel="v")
    ax4_2 = Axis(fig[2,5], xlabel="t", ylabel="Δkω")
    ax4_3 = Axis(fig[3,5], xlabel="t", ylabel="xdot_des")
    ax4_4 = Axis(fig[4,5], xlabel="t", ylabel="ydot_des")
    # ax4_5 = Axis(fig[5,5], xlabel="t", ylabel="v_des")
    lines!(ax4_1, r.ts, r.xs[3,:], label="v")
    lines!(ax4_2, r.t0_segs .- r.task_t0, r.gain_adjs[6,:], label="Δkω")
    lines!(ax4_3, r.ts, r.ctrl_setpoints[3,:], label="xdot_des")
    lines!(ax4_4, r.ts, r.ctrl_setpoints[4,:], label="ydot_des")
    ylims!(ax4_3, -3.5, 3.5)
    ylims!(ax4_4, -3.5, 3.5)
    # ylims!(ax4_5, 1.25, 2.25)

    # losses
    ax5 = Axis(fig[5,5], xlabel="Iteration", ylabel="Loss")
    lines!(ax5, losses, label="loss")

    display(fig)
end

function plot_evaluation(;
    eval_params::EvaluationParameters,
    eval_data::EvaluationData,
    algo::Union{TrainingAlgorithm, Nothing} = nothing,
    training_params::Union{TrainingParameters, Nothing} = nothing,
    sim_params::Union{SimulationParameters, Nothing} = nothing,
)
    plot_evaluation(
        eval_params.type;
        eval_params = eval_params,
        eval_data = eval_data,
        algo = algo,
        training_params = training_params,
        sim_params = sim_params,
    )
end

function plot_evaluation(
    ::UnicycleEvalType;
    eval_params::EvaluationParameters,
    eval_data::EvaluationData,
    algo::Union{TrainingAlgorithm, Nothing} = nothing,
    training_params::Union{TrainingParameters, Nothing} = nothing,
    sim_params::Union{SimulationParameters, Nothing} = nothing,
)
    fig = Figure(resolution=(1250,600))
    ax = Axis(fig[1,1:2], xlabel="x", ylabel="y")
    lines!(ax, eval_data.r.xs[1,:], eval_data.r.xs[2,:], label="Trajectory")
    lines!(ax, eval_data.r_no_model.xs[1,:], eval_data.r_no_model.xs[2,:], label="Trajectory w/o model")
    lines!(ax, eval_data.xs_task, eval_data.ys_task, label="Task", linestyle=:dash, color=:black)
    Legend(fig[1,3], ax)

    #TODO clean this up (control inputs plot) 
    fig2 = Figure(resolution=(600,600))
    ax1 = Axis(fig2[1,1], xlabel="t", ylabel="a")
    ax2 = Axis(fig2[2,1], xlabel="t", ylabel="ω")
    lines!(ax1, eval_data.r.ts, eval_data.r.us[1,:], label="accel")
    lines!(ax2, eval_data.r.ts, eval_data.r.us[2,:], label="turn rate")

    fig3 = Figure(resolution=(1000,1200))
    ax3_1 = Axis(fig3[1,1], xlabel="t", ylabel="Δkx")
    ax3_2 = Axis(fig3[2,1], xlabel="t", ylabel="Δky")
    ax3_3 = Axis(fig3[3,1], xlabel="t", ylabel="Δkv")
    ax3_4 = Axis(fig3[4,1], xlabel="t", ylabel="Δkϕ")
    ax3_5 = Axis(fig3[5,1], xlabel="t", ylabel="Δka")
    ax3_5_2 = Axis(fig3[6,1], xlabel="t", ylabel="Δkω")
    lines!(ax3_1, eval_data.r.t0_segs, eval_data.r.gain_adjs[1,:], label="Δkx")
    lines!(ax3_2, eval_data.r.t0_segs, eval_data.r.gain_adjs[2,:], label="Δky")
    lines!(ax3_3, eval_data.r.t0_segs, eval_data.r.gain_adjs[3,:], label="Δkv")
    lines!(ax3_4, eval_data.r.t0_segs, eval_data.r.gain_adjs[4,:], label="Δkϕ")
    lines!(ax3_5, eval_data.r.t0_segs, eval_data.r.gain_adjs[5,:], label="Δka")
    lines!(ax3_5_2, eval_data.r.t0_segs, eval_data.r.gain_adjs[6,:], label="Δkω")

    ax3_6 = Axis(fig3[1,2], xlabel="t", ylabel="x_sp")
    ax3_7 = Axis(fig3[2,2], xlabel="t", ylabel="y_sp")
    ax3_8 = Axis(fig3[3,2], xlabel="t", ylabel="xdot_sp")
    ax3_9 = Axis(fig3[4,2], xlabel="t", ylabel="ydot_sp")
    lines!(ax3_6, eval_data.r.t0_segs, eval_data.r.setpoints[1,:], label="x_sp")
    lines!(ax3_7, eval_data.r.t0_segs, eval_data.r.setpoints[2,:], label="y_sp")
    lines!(ax3_8, eval_data.r.t0_segs, eval_data.r.setpoints[3,:], label="xdot_sp")
    lines!(ax3_9, eval_data.r.t0_segs, eval_data.r.setpoints[4,:], label="ydot_sp")
    ax3_10 = Axis(fig3[5,2], xlabel="t", ylabel="v")
    lines!(ax3_10, eval_data.r.ts, eval_data.r.xs[3,:], label="v")

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

    display(fig)
    display(fig2)
    display(fig3)

    plot_ctrl_setpoints(eval_data.r)
end

function plot_evaluation(
    ::DoublePendulumEvalType;
    eval_params::EvaluationParameters,
    eval_data::EvaluationData,
    algo::Union{TrainingAlgorithm, Nothing} = nothing,
    training_params::Union{TrainingParameters, Nothing} = nothing,
    sim_params::Union{SimulationParameters, Nothing} = nothing,
)
    @info "dp plot eval"
end

function plot_ctrl_setpoints(r::RolloutData)
    fig = Figure(resolution=(1000,800))
    ax1 = Axis(fig[1,1], xlabel="t", ylabel="x_des")
    ax2 = Axis(fig[2,1], xlabel="t", ylabel="y_des")
    ax3 = Axis(fig[3,1], xlabel="t", ylabel="xdot_des")
    ax4 = Axis(fig[4,1], xlabel="t", ylabel="ydot_des")
    ax5 = Axis(fig[5,1], xlabel="t", ylabel="v_des")
    lines!(ax1, r.ts, r.ctrl_setpoints[1,:], label="x_des")
    lines!(ax2, r.ts, r.ctrl_setpoints[2,:], label="y_des")
    lines!(ax3, r.ts, r.ctrl_setpoints[3,:], label="xdot_des")
    lines!(ax4, r.ts, r.ctrl_setpoints[4,:], label="ydot_des")
    v = [sqrt(r.ctrl_setpoints[3,i]^2 + r.ctrl_setpoints[4,i]^2) for i in 1:length(r.ts)]
    lines!(ax5, r.ts, v, label="v_des")
    display(fig)
end

function plot_task(task::AbstractTask, sim_params::SimulationParameters)
    task_time, _ = properties(task, sim_params)
    ts = 0.0:sim_params.dt:task_time
    xs_task, ys_task, _, _ = eval_all(task, ts)
    
    fig = Figure()
    ax = Axis(fig[1,1], xlabel="x", ylabel="y")
    lines!(ax, xs_task, ys_task, label="Task")
    display(fig)
end

#TODO: separate rollouts by policy update
function animate_training(training_params::TrainingParameters, rollouts::Vector{RolloutData}, task::AbstractTask)
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
    ctrl_set_points = Observable(Point2f[(eval_data.r.ctrl_setpoints[1,1], eval_data.r.ctrl_setpoints[2,1])])
    set_points = Observable(Point2f[(eval_data.r.setpoints[1,1], eval_data.r.setpoints[2,1])])
    task_points = Observable(Point2f[(eval_data.xs_task[1], eval_data.ys_task[1])])
    task_point = Observable(Point2f[(eval_data.xs_task[1], eval_data.ys_task[1])])
    task_point_delayed = Observable(Point2f[(eval_data.xs_task[1], eval_data.ys_task[1])])
    traj_point = Observable(Point2f[(eval_data.r.xs[1,1], eval_data.r.xs[2,1])])
    T = length(eval_data.r.ts)

    fig = Figure(resolution=(850,600))
    ax = Axis(fig[1,1], xlabel="x", ylabel="y")
    lines!(ax, task_points, label="Task", linestyle=:dash, color=:black)
    lines!(ax, ctrl_set_points, label="Controller setpoints", linestyle=:dash, color=:red)
    scatter!(ax, traj_points, colormap=:thermal, markersize=7)
    scatter!(ax, set_points, colormap=:thermal, markersize=16, marker=:xcross)
    scatter!(ax, task_point, color=:black, markersize=16)
    scatter!(ax, task_point_delayed, color=:red, markersize=16)
    scatter!(ax, traj_point, color=:blue, markersize=16)
    limits!(ax, -3.5, 3.5, -2.0, 2.0)
    # limits!(ax, -6.5, 6.5, -3.5, 3.5)

    j = 1
    record(fig, path, 1:T; framterate = 100) do i
        traj_points[] = push!(traj_points[], Point2f(eval_data.r.xs[1,i], eval_data.r.xs[2,i]))
        ctrl_set_points[] = push!(ctrl_set_points[], Point2f(eval_data.r.ctrl_setpoints[1,i], eval_data.r.ctrl_setpoints[2,i]))
        if j <= length(eval_data.r.idx_segs) && i >= eval_data.r.idx_segs[j]
            set_points[] = push!(set_points[], Point2f(eval_data.r.setpoints[1,j], eval_data.r.setpoints[2,j]))
            j += 1
        end
        task_points[] = push!(task_points[], Point2f(eval_data.xs_task[i],eval_data.ys_task[i]))
        task_point[] = Point2f[(eval_data.xs_task[i],eval_data.ys_task[i])]
        delay_idx = max(1, i-Integer(round(0.0/(1/50))))
        task_point_delayed[] = Point2f[(eval_data.xs_task[delay_idx],eval_data.ys_task[delay_idx])]
        traj_point[] = Point2f[(eval_data.r.xs[1,i], eval_data.r.xs[2,i])]
        # reset_limits!(ax)
    end
end

function animate_final_evaluation(
    path, r::RolloutData, task::AbstractTask, start_perc, finish_perc, traj_color
)
    xs_task, ys_task, _, _ = eval_all(task, r.ts)

    s = Integer(ceil(start_perc*length(r.ts)))
    s_points = Integer(ceil(start_perc*length(r.setpoints[1,:])))
    f = Integer(ceil(finish_perc*length(r.ts)))
    f_points = Integer(ceil(finish_perc*length(r.setpoints[1,:])))

    traj_points = Observable(Point2f[(r.xs[1,s], r.xs[2,s])])
    set_points = Observable(Point2f[(r.setpoints[1,s_points], r.setpoints[2,s_points])])
    traj_point = Observable(Point2f[(r.xs[1,s], r.xs[2,s])])
    T = length(r.ts)

    fig = Figure(resolution=(900,600))
    ax = Axis(fig[1,1], xlabel="x (m)", ylabel="y (m)")
    lines!(ax, xs_task, ys_task, label="Task", linestyle=:dash, color=:black)
    scatter!(ax, traj_points, color=traj_color, markersize=7)
    scatter!(ax, set_points, colormap=:thermal, markersize=16, marker=:xcross, color=:black)
    scatter!(ax, traj_point, color=traj_color, markersize=16)
    limits!(ax, -3.5, 3.5, -2.5, 2.5)

    j = s_points
    record(fig, path, s:f; framterate = 100) do i
        traj_points[] = push!(traj_points[], Point2f(r.xs[1,i], r.xs[2,i]))
        if j <= length(r.idx_segs) && i >= r.idx_segs[j]
            set_points[] = push!(set_points[], Point2f(r.setpoints[1,j], r.setpoints[2,j]))
            j += 1
        end
        traj_point[] = Point2f[(r.xs[1,i], r.xs[2,i])]
    end
end

function plot_hardware_evaluation(;
    eval_params::EvaluationParameters,
    task::AbstractTask,
    algo::Union{TrainingAlgorithm, Nothing} = nothing,
    training_params::Union{TrainingParameters, Nothing} = nothing,
    sim_params::Union{SimulationParameters, Nothing} = nothing,
)
    r_model_filename = eval_params.name * "_rollout_using_model.bson"
    r_model_path = joinpath(eval_params.path, r_model_filename)
    BSON.@load r_model_path r

    r_no_model_filename = eval_params.name * "_rollout_no_model.bson"
    r_no_model_path = joinpath(eval_params.path, r_no_model_filename)
    BSON.@load r_no_model_path r_no_model

    xs_task, ys_task, _, _ = eval_all(task, r.ts .+ r.task_t0)

    
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
    # TODO need to fix this. has to do with: color = range(0.5,1.0, length=T)
    animate_evaluation(eval_params, eval_data)
end

function multi_training_plot(runs::Vector{TrainingData}, path, w, h)
    fig = Figure(resolution=(w,h))
    ax = Axis(fig[1,1], xlabel="Training Iteration", ylabel="Cost")
    for (i,run) in enumerate(runs)
        lines!(ax, run.losses, label="Trial $(i)", color=(Makie.wong_colors()[i], 1.0))
        scatter!(ax, run.losses)
    end
    axislegend(ax)
    display(fig)
    save(path, fig)
end

function final_eval_plot(path, r::RolloutData, r_no_model::RolloutData, task, w, h, start_perc, finish_perc, task_finish_perc)
    xs_task, ys_task, _, _ = eval_all(task, r.ts)
    s = Integer(round(start_perc*length(r.ts)))
    f = Integer(round(finish_perc*length(r.ts)))
    f_task = Integer(round(task_finish_perc*length(r.ts)))
    fig = Figure(resolution=(w,h))
    ax = Axis(fig[1,1], xlabel="x (m)", ylabel="y (m)")
    lines!(ax, xs_task[1:f_task], ys_task[1:f_task], label="Task", 
            color=:black, linewidth=6, linestyle=:solid)

    lines!(ax, r_no_model.xs[1,s:f], r_no_model.xs[2,s:f], label="Initial Controller", 
            color=(Makie.wong_colors()[2], .6), linewidth=4)
            lines!(ax, r.xs[1,s:f], r.xs[2,s:f], label="Learned Controller", 
            color=(Makie.wong_colors()[1], .6), linewidth=4)
    
    Legend(fig[1,2], ax)
    display(fig)
    save(path, fig)
end

function final_model_outputs_plot(
    path, r::RolloutData, task::AbstractTask, 
    ctrl_params::ControllerParameters, gain_order::Vector{<:Integer},
    w, h, start_perc, finish_perc;
    leg = nothing, gains_color=nothing
)
    # trajectory
    xs_task, ys_task, _, _ = eval_all(task, r.ts)
    s = Integer(ceil(start_perc*length(r.ts)))
    s_points = Integer(ceil(start_perc*length(r.setpoints[1,:])))
    f = Integer(ceil(finish_perc*length(r.ts)))
    f_points = Integer(ceil(finish_perc*length(r.setpoints[1,:])))

    cm = :thermal
    clrrng = (-0.2,1.4)
    f_range = range(0,1,length=f-s+1)
    f_pts_range = range(0,1,length=f_points-s_points+1)

    if isnothing(gains_color)
        g_clr = f_pts_range
    else
        g_clr = gains_color
    end

    fig = Figure(resolution=(w,h))
    ax = Axis(fig[1:3,1:3], xlabel="x (m)", ylabel="y (m)")
    scatter!(ax, r.xs[1,s:f], r.xs[2,s:f], color=f_range, colormap=cm, markersize=9, colorrange=clrrng,
            label="Trajectory")
    lines!(ax, xs_task, ys_task, label="Task", linestyle=:solid, color=:black, linewidth=2.0,)
    scatter!(ax, r.setpoints[1,s_points:f_points], r.setpoints[2,s_points:f_points], 
            marker=:xcross, color=f_pts_range, colormap=cm, markersize=16, colorrange=clrrng,
            label="Corrected setpoints")
    # axislegend(ax)

    # gains
    i=0
    for pos in gain_order
        i += 1
        if pos == 0
            continue
        end
        vpos, hpos = pos in 1:3 ? (pos, 4) : (pos-3, 5)
        tsegs = (r.t0_segs .- r.task_t0)[1:f_points]
        ts = tsegs[s_points:f_points].-tsegs[s_points]
        if ctrl_params.gains[i] != 0.0
            Ks = r.gain_adjs[i,s_points:f_points] ./ ctrl_params.gains[i] .* 100
        else
            Ks = r.gain_adjs[i,s_points:f_points] .* 100
        end
        ax2 = Axis(fig[vpos,hpos], xlabel="t (sec)", ylabel="Δ "*ctrl_params.gains_names[i]*" (%)")
        lines!(ax2, ts, Ks, color=g_clr, colormap=cm, colorrange=clrrng)
    end
    
    if !isnothing(leg)
        v,h = leg
        Legend(fig[v,h], ax)
    end

    display(fig)
    save(path, fig)
end