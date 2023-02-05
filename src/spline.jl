struct Spline
    times
    coeffs_x
    coeffs_y
    init_x
    init_y
end

function Spline(;
    x_coord,
    y_coord,
    xd_0 = 0.,
    yd_0 = 0.,
    xd_f = nothing,
    yd_f = nothing,
    times = nothing,
)
    n = length(times)
    row_length = 4*(n-1)
    A = zeros(0)
    b_x = zeros(4*(n-1))
    b_y = zeros(4*(n-1))
    init_x = x_coord[1]
    init_y = y_coord[2]

    j = 1
    for i in 0:n-1
        t = times[i+1]
        x = x_coord[i+1]
        y = y_coord[i+1]

        f′ = zeros(row_length)

        if i != 0
            idx = 4*(i-1)+1
            f = zeros(row_length)
            f[idx:idx+3] = [t^3, t^2, t, 1]
            append!(A, f)
            f′[idx:idx+2] = [3*t^2, 2*t, 1]
        else
            b_x[j:j+1] = [x, -xd_0]
            b_y[j:j+1] = [y, -yd_0]
            j += 2
        end

        if i != n-1
            idx = 4*i+1
            f = zeros(row_length)
            f[idx:idx+3] = [t^3, t^2, t, 1]
            append!(A, f)
            f′[idx:idx+2] = [-3*t^2, -2*t, -1]
        else
            b_x[j:j+1] = [x, xd_f]
            b_y[j:j+1] = [y, yd_f]
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

    return Spline(times, coeffs_x, coeffs_y, init_x, init_y)
end
