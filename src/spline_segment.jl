function spline_segment(
    ::QuadraticSpline,
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

    W = diagm([10.,10.,1.,1.])

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

function spline_segment(
    ::NoSpline,
    t0::Float64, tf::Float64,
    prev_setpoint::Vector{Float64}, setpoint::Vector{Float64}
)
    return ConstantTask(
        setpoint,
        tf
    ) 
end

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

function evaluate_segment(seg::ConstantTask, time::Real)
    return [seg.xd; [0,0]]
end

function spline_segment_cubic(
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
            t0^3    t0^2    t0  1
            tf^3    tf^2    tf  1
            3*t0^2  2*t0    1   0
            3*tf^2  2*tf    1   0
        ]

    coeffs_x = A \ [x0, xf, xdot_0, xdot_f]
    coeffs_y = A \ [y0, yf, ydot_0, ydot_f]

    return Spline(
        [t0, tf],
        coeffs_x,
        coeffs_y,
        x0,
        y0
    )
end

function evaluate_segment_cubic(spl::Spline, time::Real)
    t = time

    coeffs_x = spl.coeffs_x
    coeffs_y = spl.coeffs_y

    x = coeffs_x[1]*t^3 + coeffs_x[2]*t^2 + coeffs_x[3]*t + coeffs_x[4]
    y = coeffs_y[1]*t^3 + coeffs_y[2]*t^2 + coeffs_y[3]*t + coeffs_y[4]

    xdot = 3*coeffs_x[1]*t^2 + 2*coeffs_x[2]*t + coeffs_x[3]
    ydot = 3*coeffs_y[1]*t^2 + 2*coeffs_y[2]*t + coeffs_y[3]

    xddot = 6*coeffs_x[1]*t + 2*coeffs_x[2]
    yddot = 6*coeffs_y[1]*t + 2*coeffs_y[2]

    return [x, y, xdot, ydot, xddot, yddot]
end