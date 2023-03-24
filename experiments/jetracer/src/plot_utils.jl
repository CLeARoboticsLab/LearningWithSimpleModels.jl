function plot_rollout(
    r::RolloutData,
    task::Spline,
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
    lines!(ax3_1, r.t0_segs .- r.task_t0, r.gain_adjs[1,:], label="Δkx")
    lines!(ax3_2, r.t0_segs .- r.task_t0, r.gain_adjs[2,:], label="Δky")
    lines!(ax3_3, r.t0_segs .- r.task_t0, r.gain_adjs[3,:], label="Δkv")
    lines!(ax3_4, r.t0_segs .- r.task_t0, r.gain_adjs[4,:], label="Δkϕ")

    # control setpoints
    vs = [sqrt(r.ctrl_setpoints[3,i]^2 + r.ctrl_setpoints[4,i]^2) for i in 1:length(r.ts)]
    ax4_1 = Axis(fig[1,5], xlabel="t", ylabel="x_des")
    ax4_2 = Axis(fig[2,5], xlabel="t", ylabel="y_des")
    ax4_3 = Axis(fig[3,5], xlabel="t", ylabel="xdot_des")
    ax4_4 = Axis(fig[4,5], xlabel="t", ylabel="ydot_des")
    ax4_5 = Axis(fig[5,5], xlabel="t", ylabel="v_des")
    lines!(ax4_1, r.ts, r.ctrl_setpoints[1,:], label="x_des")
    lines!(ax4_2, r.ts, r.ctrl_setpoints[2,:], label="y_des")
    lines!(ax4_3, r.ts, r.ctrl_setpoints[3,:], label="xdot_des")
    lines!(ax4_4, r.ts, r.ctrl_setpoints[4,:], label="ydot_des")
    lines!(ax4_5, r.ts, vs, label="v_des")
    ylims!(ax4_1, -4, 4)
    ylims!(ax4_2, -2, 2)
    ylims!(ax4_3, -2.5, 2.5)
    ylims!(ax4_4, -2.5, 2.5)
    ylims!(ax4_5, 1.25, 2.25)

    # losses
    ax5 = Axis(fig[5,4], xlabel="Iteration", ylabel="Loss")
    lines!(ax5, losses, label="loss")

    display(fig)
end