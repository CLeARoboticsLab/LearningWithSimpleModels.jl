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
    A_x = zeros(0)
    A_y = zeros(0)
    b_x = zeros(4*(n-1))
    b_y = zeros(4*(n-1))
    init_x = x_coord[1]
    init_y = y_coord[2]

    j = 1
    for i in 0:n-1
        t = times[i+1]
        x = x_coord[i+1]
        y = y_coord[i+1]

        f_x_prime = zeros(row_length)
        f_y_prime = zeros(row_length)

        if i != 0
            f_x = zeros(row_length)
            f_x[4*(i-1)+1] = t^3
            f_x[4*(i-1) + 1+1] = t^2
            f_x[4*(i-1) + 2+1] = t
            f_x[4*(i-1) + 3+1] = 1
            append!(A_x, f_x)

            f_x_prime[4*(i-1)+1] = 3*t^2
            f_x_prime[4*(i-1) + 1+1] = 2*t
            f_x_prime[4*(i-1) + 2+1] = 1

            f_y = zeros(row_length)
            f_y[4*(i-1)+1] = t^3
            f_y[4*(i-1) + 1+1] = t^2
            f_y[4*(i-1) + 2+1] = t
            f_y[4*(i-1) + 3+1] = 1
            append!(A_y, f_y)

            f_y_prime[4*(i-1)+1] = 3*t^2
            f_y_prime[4*(i-1) + 1+1] = 2*t
            f_y_prime[4*(i-1) + 2+1] = 1
        else
            b_x[j] = x
            b_x[j+1] = -xd_0
            b_y[j] = y
            b_y[j+1] = -yd_0
            j += 2
        end

        if i != n-1
            f_x = zeros(row_length)
            f_x[4*i+1] = t^3
            f_x[4*i + 1+1] = t^2
            f_x[4*i + 2+1] = t
            f_x[4*i + 3+1] = 1
            append!(A_x, f_x)

            f_x_prime[4*i+1] = -3*t^2
            f_x_prime[4*i + 1+1] = -2*t
            f_x_prime[4*i + 2+1] = -1

            f_y = zeros(row_length)
            f_y[4*i+1] = t^3
            f_y[4*i + 1+1] = t^2
            f_y[4*i + 2+1] = t
            f_y[4*i + 3+1] = 1
            append!(A_y, f_y)

            f_y_prime[4*i+1] = -3*t^2
            f_y_prime[4*i + 1+1] = -2*t
            f_y_prime[4*i + 2+1] = -1
        else
            b_x[j] = x
            b_x[j+1] = xd_f
            b_y[j] = y
            b_y[j+1] = yd_f
            j += 2
        end

        append!(A_x, f_x_prime)
        append!(A_y, f_y_prime)

        if i != 0 && i != n-1
            #second derivative rows
            f_x_dprime = zeros(row_length)
            f_x_dprime[4*(i-1)+1] = 6*t
            f_x_dprime[4*(i-1) + 1+1] = 2

            f_x_dprime[4*i+1] = -6*t
            f_x_dprime[4*i + 1+1] = -2

            append!(A_x, f_x_dprime)

            f_y_dprime = zeros(row_length)
            f_y_dprime[4*(i-1)+1] = 6*t
            f_y_dprime[4*(i-1) + 1+1] = 2

            f_y_dprime[4*i+1] = -6*t
            f_y_dprime[4*i + 1+1] = -2

            append!(A_y, f_y_dprime)

            b_x[j] = x
            b_x[j+1] = x
            b_x[j+2] = 0
            b_x[j+3] = 0
            b_y[j] = y
            b_y[j+1] = y
            b_y[j+2] = 0
            b_y[j+3] = 0
            j += 4
        end

    end

    A_x = reshape(A_x, (row_length, row_length))'
    A_y = reshape(A_y, (row_length, row_length))'

    coeffs_x = inv(A_x)*b_x
    coeffs_y = inv(A_y)*b_y

    return Spline(times, coeffs_x, coeffs_y, init_x, init_y)
end

