function plot_losses(losses)
    fig = Figure()
    ax = Axis(fig[1,1], xlabel="Iteration", ylabel="Loss")
    lines!(ax, losses)
    display(GLMakie.Screen(), fig)
end

function plot_evaluation(eval_data::EvaluationData)
    fig = Figure()
    ax = Axis(fig[1,1], xlabel="x", ylabel="y")
    lines!(ax, eval_data.xs[1,:], eval_data.xs[2,:], label="Trajectory")
    lines!(ax, eval_data.xs_no_model[1,:], eval_data.xs_no_model[2,:], label="Trajectory w/o model")
    lines!(ax, eval_data.xs_task, eval_data.ys_task, label="Task", linestyle=:dash, color=:black)
    Legend(fig[1,2], ax)
    display(GLMakie.Screen(), fig)
end