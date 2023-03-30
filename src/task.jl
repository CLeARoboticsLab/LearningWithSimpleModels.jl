function wrapped_time(task::AbstractTask, time::Real)
    t = time
    task_time = end_time(task)
    if time > task_time
        t = time - (time ÷ task_time)*task_time
    end
    return t
end

# tasks in a point evaluated from a task in the form [x, y, xdot, ydot] and
# replaces xdot and ydot with velocity and heading angle, respectively
function to_velocity_and_heading_angle(task_point::Vector{Float64})
    xdot = task_point[3]
    ydot = task_point[4]
    
    v = sqrt(xdot^2 + ydot^2)

    if xdot == 0
        ϕ = sign(ydot)*π/2
    else
        ϕ = atan(ydot,xdot)
    end

    return [task_point[1], task_point[2], v, ϕ]
end

function plot_task(task)
    ts = 0.0:0.01:30.0
    xs, ys, xdots, ydots = eval_all(task, ts)
    vs = zeros(length(ts))
    # as = zeros(length(ts))
    for i in 1:length(ts)
        vs[i] = sqrt(xdots[i]^2 + ydots[i]^2)
        # as[i] = sqrt(xddots[i]^2 + yddots[i]^2)
    end
    fig = Figure(resolution=(800,800))
    ax = Axis(fig[1,1], xlabel="t", ylabel="xdot")
    ax2 = Axis(fig[2,1], xlabel="t", ylabel="ydot")
    ax3 = Axis(fig[3,1], xlabel="t", ylabel="v")
    lines!(ax, ts, xdots)
    lines!(ax2, ts, ydots)
    lines!(ax3, ts, vs)

    fig2 = Figure(resolution=(850,600))
    ax3 = Axis(fig2[1,1], xlabel="x", ylabel="y")
    lines!(ax3, xs, ys, label="Task", linestyle=:dash, color=:black)

    display(fig)
    display(fig2)
end