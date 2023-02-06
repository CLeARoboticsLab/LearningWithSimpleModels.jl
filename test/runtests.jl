using LearningWithSimpleModels
using Test

@testset verbose = true "LearningWithSimpleModels.jl" begin
    
    @testset "Spline" begin
        
        # evaluate the spline at all the test_ts and return arrays of the results
        function eval_all(spl, test_ts)
            xs_spline = zeros(length(test_ts))
            ys_spline = zeros(length(test_ts))
            xdots_spline = zeros(length(test_ts))
            ydots_spline = zeros(length(test_ts))
            for (i,t) in enumerate(test_ts)
                xs_spline[i], ys_spline[i], xdots_spline[i], ydots_spline[i] = evaluate(spl, t)
            end
            return xs_spline, ys_spline, xdots_spline, ydots_spline
        end

        let times = [0., 1., 2., 3., 4.]
            coeffs_x = [2., -3., -4., 2.]
            coeffs_y = rand(4)
            cubic_func(coeffs) = (t) -> coeffs[1]*t^3 + coeffs[2]*t^2 + coeffs[3]*t + coeffs[4]
            cubic_func_dot(coeffs) = (t) -> 3*coeffs[1]*t^2 + 2*coeffs[2]*t + coeffs[3]
            fx = cubic_func(coeffs_x)
            fy = cubic_func(coeffs_y)
            fx_dot = cubic_func_dot(coeffs_x)
            fy_dot = cubic_func_dot(coeffs_y)
            xs = fx.(times)
            ys = fy.(times)

            spl = Spline(;
                xs = xs,
                ys = ys,
                ts = times,
                xdot_0 = fx_dot(times[1]),
                ydot_0 = fy_dot(times[1]),
                xdot_f = fx_dot(times[end]),
                ydot_f = fy_dot(times[end]),
            )
            @test all(spl.coeffs_x[1:4] .≈ coeffs_x)
            @test all(spl.coeffs_y[1:4] .≈ coeffs_y)

            test_ts = 0.0:.1:4.0
            xs_spline, ys_spline, xdots_spline, ydots_spline = eval_all(spl, test_ts)
            @test all(xs_spline .≈ fx.(test_ts))
            @test all(ys_spline .≈ fy.(test_ts))
            @test all(xdots_spline .≈ fx_dot.(test_ts))
            @test all(ydots_spline .≈ fy_dot.(test_ts))
        end
        
        let ts = [0.0, .7, 1.5, 2.5, 3.0, 4.5]
            t = 0.0
            @test LearningWithSimpleModels.time_segment(t, ts) == 1
            t = 2.6
            @test LearningWithSimpleModels.time_segment(t, ts) == 4
            t = 4.5
            @test LearningWithSimpleModels.time_segment(t, ts) == 5
        end

        let time = 10.0 + 5.0*(rand()-.5)
            r = 2.0 + .5*(rand()-.5)
            x0 = 10.0*(rand()-.5)
            y0 = 5.0*(rand()-.5)
            spl = figure_eight(;
                x0 = x0,
                y0 = y0,
                xdot_0 = 0.0,
                ydot_0 = 0.0,
                xdot_f = 0.0,
                ydot_f = 0.0,
                radius = r,
                time = time
            )
            atol = r/10
            test_ts = 0:time/16:time
            xs_spline, ys_spline, xdots_spline, ydots_spline = eval_all(spl, test_ts)
            println("$(xs_spline[4]) ; $(x0+r+r*cos(π/4))")
            @test all([
                isapprox(xs_spline[4], x0+r+r*cos(π/4); atol=atol),
                isapprox(xs_spline[6], x0+r+r*cos(π/4); atol=atol),
                isapprox(xs_spline[12], x0-r-r*cos(π/4); atol=atol),
                isapprox(xs_spline[14], x0-r-r*cos(π/4); atol=atol)
            ])
            @test all([
                isapprox(ys_spline[4], y0+r*sin(π/4); atol=atol),
                isapprox(ys_spline[6], y0-r*sin(π/4); atol=atol),
                isapprox(ys_spline[12], y0+r*sin(π/4); atol=atol),
                isapprox(ys_spline[14], y0-r*sin(π/4); atol=atol)
            ])
        end
    end
end
