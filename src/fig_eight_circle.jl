function FigEightCircle(;r::Real=3.0, time::Real=10.0)
    c = 2*π*r
    v = 2*c/time
    return FigEightCircle(r, time, v)
end

function evaluate(cir::FigEightCircle, time::Real; wrap_time::Bool=true)
    t = wrap_time ? wrapped_time(cir, time) : time
    if t < cir.time/2
        x = -cir.r*cos(4*π*t/cir.time) + cir.r
        y = cir.r*sin(4*π*t/cir.time)
        xdot = 4*π*cir.r/cir.time*sin(4*π*t/cir.time)
        ydot = 4*π*cir.r/cir.time*cos(4*π*t/cir.time)
    else
        x = cir.r*cos(4*π*t/cir.time) - cir.r
        y = cir.r*sin(4*π*t/cir.time)
        xdot = -4*π*cir.r/cir.time*sin(4*π*t/cir.time)
        ydot = 4*π*cir.r/cir.time*cos(4*π*t/cir.time)
    end
    return [x, y, xdot, ydot]
end

end_time(cir::FigEightCircle) = cir.time

function test_plot_fig_eight_circle()
    cir = FigEightCircle()
    plot_task(cir)
end

function wrapped_time(task::AbstractTask, time::Real)
    t = time
    task_time = end_time(task)
    if time > task_time
        t = time - (time ÷ task_time)*task_time
    end
    return t
end