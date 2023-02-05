using LearningWithSimpleModels
using Test

@testset verbose = true "LearningWithSimpleModels.jl" begin
    
    @testset "Spline" begin
        
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
            xs_spline = zeros(length(test_ts))
            ys_spline = zeros(length(test_ts))
            xdots_spline = zeros(length(test_ts))
            ydots_spline = zeros(length(test_ts))
            for (i,t) in enumerate(test_ts)
                xs_spline[i], ys_spline[i], xdots_spline[i], ydots_spline[i] = evaluate(spl, t)
            end
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

        #TODO test w/ figure 8
        spl = Spline(;
            xs = [0., 3, 6, 3, 0, -3, -6, -3, 0],
            ys = [0., 3, 0, -3, 0, 3, 0, -3, 0],
            ts = [ 0., 1.25, 2.5, 3.75, 5. , 6.25, 7.5 , 8.75, 10.] #,
            # xdot_f = 2.4,
            # ydot_f = 2.4,
        )
    end
end
