"""
Generate a cubic spline
# Arguments
- `xs`, `ys`: x- and y-coordinates
- `ts`: arrival time at each coordinate (first point must be >= 0.0)
- `xdot_0`, `ydot_0`: x and y initial velocities
- `xdot_f`, `ydot_f`: x and y final velocities; if nothing, caluclates from last two points
"""
function Spline(;
    xs::Vector{Float64},
    ys::Vector{Float64},
    ts::Vector{Float64},
    xdot_0 = nothing,
    ydot_0 = nothing,
    xdot_f = nothing,
    ydot_f = nothing,
)
    if isnothing(xdot_0)
        xdot_0 = (xs[2] - xs[1]) / (ts[2] - ts[1])
    end
    if isnothing(ydot_0)
        ydot_0 = (ys[2] - ys[1]) / (ts[2] - ts[1])
    end
    if isnothing(xdot_f)
        xdot_f = (xs[end] - xs[end-1]) / (ts[end] - ts[end-1])
    end
    if isnothing(ydot_f)
        ydot_f = (ys[end] - ys[end-1]) / (ts[end] - ts[end-1])
    end

    n = length(ts)
    row_length = 8*(n-1)
    A = zeros(0)
    b_x = zeros(8*(n-1))
    b_y = zeros(8*(n-1))
    x0 = xs[1]
    y0 = ys[1]

    j = 1
    for i in 0:n-1
        t = ts[i+1]
        x = xs[i+1]
        y = ys[i+1]

        f2 = zeros(row_length)
        f6 = zeros(row_length)
        f7 = zeros(row_length)

        if i != 0
            idx = 8*(i-1)+1
            f = zeros(row_length)
            f[idx:idx+7] = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]
            append!(A, f)
            if i == n-1
                f2[idx:idx+5] = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2]
                f6[idx:idx+1] = [5040*t, 720]
                f7[idx] = 5040
                append!(A, f2)
                append!(A, f6)
                append!(A, f7)
            end
        else
            b_x[j:j+3] = [x, 0.0, 0.0, 0.0]
            b_y[j:j+3] = [y, 0.0, 0.0, 0.0]
            j += 4
        end

        if i != n-1
            idx = 8*i+1
            f = zeros(row_length)
            f[idx:idx+7] = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]
            append!(A, f)
            if i == 0
                f2[idx:idx+5] = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2]
                f6[idx:idx+1] = [5040*t, 720]
                f7[idx] = 5040
                append!(A, f2)
                append!(A, f6)
                append!(A, f7)
            end
        else
            b_x[j:j+3] = [x, 0.0, 0.0, 0.0]
            b_y[j:j+3] = [y, 0.0, 0.0, 0.0]
            j += 4
        end

        if i != 0 && i != n-1
            #1st order derivative row
            f′ = zeros(row_length)
            f′[8*(i-1)+1 : 8*(i-1)+1+6] = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1]
            f′[8*(i)+1 : 8*(i)+1+6] = [-7*t^6, -6*t^5, -5*t^4, -4*t^3, -3*t^2, -2*t, -1]
            append!(A, f′)

            #2nd order derivative row
            f′′ = zeros(row_length)
            f′′[8*(i-1)+1 : 8*(i-1)+1+5] = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2]
            f′′[8*(i)+1 : 8*(i)+1+5] = [-42*t^5, -30*t^4, -20*t^3, -12*t^2, -6*t, -2]
            append!(A, f′′)

            #3rd order derivative row
            f′′′ = zeros(row_length)
            f′′′[8*(i-1)+1 : 8*(i-1)+1+4] = [210*t^4, 120*t^3, 60*t^2, 24*t, 6]
            f′′′[8*(i)+1 : 8*(i)+1+4] = [-210*t^4, -120*t^3, -60*t^2, -24*t, -6]
            append!(A, f′′′)

            #4th order derivative row
            f4 = zeros(row_length)
            f4[8*(i-1)+1 : 8*(i-1)+1+3] = [840*t^3, 360*t^2, 120*t, 24]
            f4[8*(i)+1 : 8*(i)+1+3] = [-840*t^3, -360*t^2, -120*t, -24]
            append!(A, f4)

            #5th order derivative row
            f5 = zeros(row_length)
            f5[8*(i-1)+1 : 8*(i-1)+1+2] = [2520*t^2, 720*t, 120]
            f5[8*(i)+1 : 8*(i)+1+2] = [-2520*t^2, -720*t, -120]
            append!(A, f5)

            #6th order derivative row
            f6 = zeros(row_length)
            f6[8*(i-1)+1 : 8*(i-1)+1+1] = [5040*t, 720]
            f6[8*(i)+1 : 8*(i)+1+1] = [-5040*t, -720]
            append!(A, f6)

            b_x[j:j+7] = [x, x, 0, 0, 0, 0, 0, 0]
            b_y[j:j+7] = [y, y, 0, 0, 0, 0, 0, 0]
            j += 8
        end
    end

    A = reshape(A, (row_length, row_length))'
    coeffs_x = inv(A)*b_x
    coeffs_y = inv(A)*b_y

    return Spline(ts, coeffs_x, coeffs_y, x0, y0)
end

function time_segment(t::Float64, ts::Vector{Float64})
    if length(ts) == 2
        return 1
    elseif t == ts[1]
        return 1
    end
    return findall(ts .- t .< 0.0)[end]
end

"""
Returns a vector of x, y, xdot, ydot for the Spline at time specified.
"""
function evaluate(spl::Spline, time::Real; wrap_time::Bool=true)
    t = wrap_time ? wrapped_time(spl, time) : time

    seg = time_segment(t, spl.ts)

    idx = 8*(seg-1) + 1
    coeffs_x = spl.coeffs_x[idx:idx+7]
    coeffs_y = spl.coeffs_y[idx:idx+7]

    x = coeffs_x[1]*t^7 + coeffs_x[2]*t^6 + coeffs_x[3]*t^5 + coeffs_x[4]*t^4 + coeffs_x[5]*t^3 + coeffs_x[6]*t^2 + coeffs_x[7]*t + coeffs_x[8]
    y = coeffs_y[1]*t^7 + coeffs_y[2]*t^6 + coeffs_y[3]*t^5 + coeffs_y[4]*t^4 + coeffs_y[5]*t^3 + coeffs_y[6]*t^2 + coeffs_y[7]*t + coeffs_y[8]

    xdot = 7*coeffs_x[1]*t^6 + 6*coeffs_x[2]*t^5 + 5*coeffs_x[3]*t^4 + 4*coeffs_x[4]*t^3 + 3*coeffs_x[5]*t^2 + 2*coeffs_x[6]*t + coeffs_x[7]
    ydot = 7*coeffs_y[1]*t^6 + 6*coeffs_y[2]*t^5 + 5*coeffs_y[3]*t^4 + 4*coeffs_y[4]*t^3 + 3*coeffs_y[5]*t^2 + 2*coeffs_y[6]*t + coeffs_y[7]
    
    xddot = 6*7*coeffs_x[1]*t^5 + 5*6*coeffs_x[2]*t^4 + 4*5*coeffs_x[3]*t^3 + 3*4*coeffs_x[4]*t^2 + 2*3*coeffs_x[5]*t + 2*coeffs_x[6]
    yddot = 6*7*coeffs_y[1]*t^5 + 5*6*coeffs_y[2]*t^4 + 4*5*coeffs_y[3]*t^3 + 3*4*coeffs_y[4]*t^2 + 2*3*coeffs_y[5]*t + 2*coeffs_y[6]

    return [x, y, xdot, ydot]
end

# evaluate the spline at all the test_ts and return arrays of the results
function eval_all(spl, test_ts)
    xs_spline = zeros(length(test_ts))
    ys_spline = zeros(length(test_ts))
    xdots_spline = zeros(length(test_ts))
    ydots_spline = zeros(length(test_ts))
    xddots_spline = zeros(length(test_ts))
    yddots_spline = zeros(length(test_ts))
    for (i,t) in enumerate(test_ts)
        xs_spline[i], ys_spline[i], xdots_spline[i], ydots_spline[i] = evaluate(spl, t)
    end
    return xs_spline, ys_spline, xdots_spline, ydots_spline
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

end_time(spl::Spline) = last(spl.ts)

function properties(task::AbstractTask, sim_params::SimulationParameters)
    task_time = end_time(task)
    segment_length = Integer(round(sim_params.model_dt / sim_params.dt))
    return task_time, segment_length
end

"""
Returns a cubic spline in the shape of a sideways figure eight
# Arguments
- `x0`, `y0`: center of the figure eight
- `xdot_0`, `ydot_0`: x and y initial velocities
- `xdot_f`, `ydot_f`: x and y final velocities
- `radius`: approximate radius of the two loops
- `time`: time to complete the figure eight
"""
function figure_eight(;
    x0::Float64 = 0.0,
    y0::Float64 = 0.0,
    xdot_0 = nothing,
    ydot_0 = nothing,
    xdot_f = nothing,
    ydot_f = nothing,
    radius::Float64 = 1.0,
    time::Float64 = 8.0,
    laps::Integer = 1
)
    xs_fig_eight = [0, radius, 2*radius, radius, 0, -radius, -2*radius, -radius]
    ys_fig_eight = [0, radius, 0, -radius, 0, radius, 0, -radius]
    n_pts = length(xs_fig_eight)

    xs = zeros(laps*8 + 1)
    ys = zeros(laps*8 + 1)

    for i in 1:laps
        start_idx = (i-1)*n_pts + 1
        end_idx = (i-1)*n_pts + n_pts
        xs[start_idx:end_idx] = xs_fig_eight
        ys[start_idx:end_idx] = ys_fig_eight
    end

    xs = xs .+ x0
    ys = ys .+ y0

    interval = time/8
    ts = collect(0.0:interval:time*laps)

    return Spline(;
        xs = xs,
        ys = ys,
        ts = ts,
        xdot_0 = xdot_0,
        ydot_0 = ydot_0,
        xdot_f = xdot_f,
        ydot_f = ydot_f,
    )
end

function spline_segment(
    t0::Float64, tf::Float64,
    prev_setpoint::Vector{Float64}, setpoint::Vector{Float64}
)
    
    x0 = prev_setpoint[1]
    xf = setpoint[1]
    xdot_0 = prev_setpoint[3]
    xdot_f = setpoint[3]

    y0 = prev_setpoint[2]
    yf = setpoint[2]
    ydot_0 = prev_setpoint[4]
    ydot_f = setpoint[4]

    A = [
            t0^2    t0  1
            tf^2    tf  1
            2*t0    1   0
            2*tf    1   0
        ]

    W = diagm([1.,1.,1.,1.])

    coeffs_x = inv(A'*W*A)*A'*W*[x0, xf, xdot_0, xdot_f]
    coeffs_y = inv(A'*W*A)*A'*W*[y0, yf, ydot_0, ydot_f]

    return Spline(
        [t0, tf],
        coeffs_x,
        coeffs_y,
        x0,
        y0
    )
end

# function evaluate_segment(spl::Spline, time::Real)
#     t = time

#     coeffs_x = spl.coeffs_x
#     coeffs_y = spl.coeffs_y

#     x = coeffs_x[1]*t^5 + coeffs_x[2]*t^4 + coeffs_x[3]*t^3 + coeffs_x[4]*t^2 + coeffs_x[5]*t + coeffs_x[6]
#     y = coeffs_y[1]*t^5 + coeffs_y[2]*t^4 + coeffs_y[3]*t^3 + coeffs_y[4]*t^2 + coeffs_y[5]*t + coeffs_y[6]

#     xdot = 5*coeffs_x[1]*t^4 + 4*coeffs_x[2]*t^3 + 3*coeffs_x[3]*t^2 + 2*coeffs_x[4]*t + coeffs_x[5]
#     ydot = 5*coeffs_y[1]*t^4 + 4*coeffs_y[2]*t^3 + 3*coeffs_y[3]*t^2 + 2*coeffs_y[4]*t + coeffs_y[5]

#     return [x, y, xdot, ydot]
# end

function evaluate_segment(spl::Spline, time::Real)
    t = time

    coeffs_x = spl.coeffs_x
    coeffs_y = spl.coeffs_y

    x = coeffs_x[1]*t^2 + coeffs_x[2]*t + coeffs_x[3]
    y = coeffs_y[1]*t^2 + coeffs_y[2]*t + coeffs_y[3]

    xdot = 2*coeffs_x[1]*t + coeffs_x[2]
    ydot = 2*coeffs_y[1]*t + coeffs_y[2]

    xddot = 2*coeffs_x[1]
    yddot = 2*coeffs_y[1]

    return [x, y, xdot, ydot, xddot, yddot]
end

function test_plot_acceleration()
    spl = figure_eight(;
        x0 = 0.0,
        y0 = 0.0,
        xdot_0 = nothing,
        ydot_0 = nothing,
        xdot_f = nothing,
        ydot_f = nothing,
        radius = 3.00,
        time = 10.0,
        laps = 1
    )
    plot_task(spl)
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

    display(GLMakie.Screen(), fig)
    display(GLMakie.Screen(), fig2)
end