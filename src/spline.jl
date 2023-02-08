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
    xdot_0::Float64 = 0.,
    ydot_0::Float64 = 0.,
    xdot_f = nothing,
    ydot_f = nothing,
)
    if isnothing(xdot_f)
        xdot_f = (xs[end] - xs[end-1]) / (ts[end] - ts[end-1])
    end
    if isnothing(ydot_f)
        ydot_f = (ys[end] - ys[end-1]) / (ts[end] - ts[end-1])
    end

    n = length(ts)
    row_length = 4*(n-1)
    A = zeros(0)
    b_x = zeros(4*(n-1))
    b_y = zeros(4*(n-1))
    x0 = xs[1]
    y0 = ys[1]

    j = 1
    for i in 0:n-1
        t = ts[i+1]
        x = xs[i+1]
        y = ys[i+1]

        f′ = zeros(row_length)

        if i != 0
            idx = 4*(i-1)+1
            f = zeros(row_length)
            f[idx:idx+3] = [t^3, t^2, t, 1]
            append!(A, f)
            f′[idx:idx+2] = [3*t^2, 2*t, 1]
        else
            b_x[j:j+1] = [x, -xdot_0]
            b_y[j:j+1] = [y, -ydot_0]
            j += 2
        end

        if i != n-1
            idx = 4*i+1
            f = zeros(row_length)
            f[idx:idx+3] = [t^3, t^2, t, 1]
            append!(A, f)
            f′[idx:idx+2] = [-3*t^2, -2*t, -1]
        else
            b_x[j:j+1] = [x, xdot_f]
            b_y[j:j+1] = [y, ydot_f]
            j += 2
        end

        append!(A, f′)

        if i != 0 && i != n-1
            #second derivative rows
            f′′ = zeros(row_length)
            f′′[4*(i-1)+1] = 6*t
            f′′[4*(i-1) + 1+1] = 2

            f′′[4*i+1] = -6*t
            f′′[4*i + 1+1] = -2

            append!(A, f′′)

            b_x[j:j+3] = [x, x, 0, 0]
            b_y[j:j+3] = [y, y, 0, 0]
            j += 4
        end
    end

    A = reshape(A, (row_length, row_length))'
    coeffs_x = inv(A)*b_x
    coeffs_y = inv(A)*b_y

    return Spline(ts, coeffs_x, coeffs_y, x0, y0)
end

function time_segment(t::Float64, ts::Vector{Float64})
    if t == ts[1]
        return 1
    end
    return findall(ts .- t .< 0.0)[end]
end

"""
Returns a vector of x, y, xdot, ydot for the Spline at time t.
"""
function evaluate(spl::Spline, t::Real)
    seg = time_segment(t, spl.ts)

    idx = 4*(seg-1) + 1
    coeffs_x = spl.coeffs_x[idx:idx+3]
    coeffs_y = spl.coeffs_y[idx:idx+3]

    x = coeffs_x[1]*t^3 + coeffs_x[2]*t^2 + coeffs_x[3]*t + coeffs_x[4]
    y = coeffs_y[1]*t^3 + coeffs_y[2]*t^2 + coeffs_y[3]*t + coeffs_y[4]

    xdot = 3*coeffs_x[1]*t^2 + 2*coeffs_x[2]*t + coeffs_x[3]
    ydot = 3*coeffs_y[1]*t^2 + 2*coeffs_y[2]*t + coeffs_y[3]

    return [x, y, xdot, ydot]
end

end_time(spl::Spline) = last(spl.ts)

function properties(task::Spline, params::TrainingParameters)
    task_time = end_time(task)
    n_segments = Integer(round(task_time/params.model_dt))
    segment_length = Integer(round(params.model_dt / params.dt))
    return task_time, n_segments, segment_length
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
    xdot_0::Float64 = 0.0,
    ydot_0::Float64 = 0.0,
    xdot_f = nothing,
    ydot_f = nothing,
    radius::Float64 = 1.0,
    time::Float64 = 8.0
)
    xs = [0, radius, 2*radius, radius, 0, -radius, -2*radius, -radius, 0]
    ys = [0, radius, 0, -radius, 0, radius, 0, -radius, 0]

    xs = xs .+ x0
    ys = ys .+ y0

    interval = time/8
    ts = collect(0.0:interval:time)

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