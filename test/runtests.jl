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

            spl = LearningWithSimpleModels.Spline(;
                x_coord = xs[2:end],
                y_coord = ys[2:end],
                xd_0 = fx_dot(times[1]),
                yd_0 = fy_dot(times[1]),
                xd_f = fx_dot(times[end]),
                yd_f = fy_dot(times[end]),
                times = times,
                init_pos = [fx(times[1]), fy(times[1])]
            )
            @test all(spl.coeffs_x[1:4] .≈ coeffs_x)
            @test all(spl.coeffs_y[1:4] .≈ coeffs_y)
        end
        
        #TODO test w/ figure 8
        spl = LearningWithSimpleModels.Spline(;
            x_coord = [3, 6, 3, 0, -3, -6, -3, 0],
            y_coord = [3, 0, -3, 0, 3, 0, -3, 0],
            xd_f = 2.4,
            yd_f = 2.4,
            times = [ 0., 1.25, 2.5, 3.75, 5. , 6.25, 7.5 , 8.75, 10.],
            init_pos = [0., 0.]
        )
    end
end
