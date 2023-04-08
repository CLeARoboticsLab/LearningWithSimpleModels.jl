function make_training_plot()
    base_path = ".data/JetRacer/training_data/1.00e-3_lr"
    paths = [
        joinpath(base_path, "jetracer_training_data_1.bson"),
        joinpath(base_path, "jetracer_training_data_2.bson"),
        joinpath(base_path, "jetracer_training_data_3.bson")
    ]
    runs = Vector{TrainingData}()
    for path in paths
        run = BSON.load(path)
        push!(runs, run[:data])
    end

    multi_training_plot(runs,".data/JetRacer/losses_large.png",800,600)
    multi_training_plot(runs,".data/JetRacer/losses_small.png",600,400)

end

function multi_training_plot(runs, path, w, h)
    fig = Figure(resolution=(w,h))
    ax = Axis(fig[1,1], xlabel="Episode", ylabel="Loss")
    for (i,run) in enumerate(runs)
        lines!(ax, run.losses, label="Epoch $(i)", colormap = (:viridis, 0.4))
        scatter!(ax, run.losses)
    end
    axislegend(ax)
    display(fig)
    save(path, fig)
end

function make_final_eval_plot()
    base_path = ".data/JetRacer/runs/2_1.0lr"
    r = BSON.load(joinpath(base_path,"jetracer_rollout_using_model.bson"))[:r]
    r_no_model = BSON.load(joinpath(base_path,"jetracer_rollout_no_model.bson"))[:r_no_model]
    task = jetracer_figure_eight_task()
    path = ".data/JetRacer/trajectory.png"
    final_eval_plot(path, r, r_no_model, task, 800, 450, .42, .99)
end

function final_eval_plot(path, r, r_no_model, task, w, h, start, finish)
    xs_task, ys_task, _, _ = eval_all(task, r.ts)
    s = Integer(round(start*length(r.ts)))
    f = Integer(round(finish*length(r.ts)))
    f_task = Integer(round((1/6.42)*length(r.ts)))
    T = f-s+1
    fig = Figure(resolution=(w,h))
    ax = Axis(fig[1,1], xlabel="x (m)", ylabel="y (m)")
    lines!(ax, xs_task[1:f_task], ys_task[1:f_task], label="Task", color=:black, linewidth=6, linestyle=:dash)
    # scatter!(ax, r_no_model.xs[1,s:f], r_no_model.xs[2,s:f], label="Untrained", markersize=7, 
    #         color=(Makie.wong_colors()[2], .5))
    # scatter!(ax, r.xs[1,s:f], r.xs[2,s:f], label="Trained", markersize=7, 
    #         color=(Makie.wong_colors()[1], .5))
    
    # scatter!(ax, r_no_model.xs[1,s:f], r_no_model.xs[2,s:f], label="Untrained", markersize=7, color=range(0.5,1.0, length=T), colormap=(:Purples_6,0.6))
    # scatter!(ax, r.xs[1,s:f], r.xs[2,s:f], label="Trained", markersize=7, color=range(0.5,1.0, length=T), colormap=(:Reds_9,0.6))


    lines!(ax, r_no_model.xs[1,s:f], r_no_model.xs[2,s:f], label="Untrained", 
            color=(Makie.wong_colors()[2], .6), linewidth=4)
            lines!(ax, r.xs[1,s:f], r.xs[2,s:f], label="Trained", 
            color=(Makie.wong_colors()[1], .6), linewidth=4)
    
    axislegend(ax, position = :rb)
    display(fig)
    save(path, fig)
end

