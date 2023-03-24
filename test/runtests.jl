using LearningWithSimpleModels
using Test

include("../experiments/unicycle/unicycle_experiment.jl")

@testset verbose = true "LearningWithSimpleModels.jl" begin
    
    @testset "Spline" begin       
        let ts = [0.0, .7, 1.5, 2.5, 3.0, 4.5]
            t = 0.0
            @test LearningWithSimpleModels.time_segment(t, ts) == 1
            t = 2.6
            @test LearningWithSimpleModels.time_segment(t, ts) == 4
            t = 4.5
            @test LearningWithSimpleModels.time_segment(t, ts) == 5
        end
    end

    @testset "Figure Eight" begin
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
            xs_spline, ys_spline, xdots_spline, ydots_spline = LearningWithSimpleModels.eval_all(spl, test_ts)

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

        let time = 10.0, laps = 3
            r = 2.0
            atol = r/100
            spl = figure_eight(;
                xdot_0 = nothing,
                ydot_0 = nothing,
                xdot_f = nothing,
                ydot_f = nothing,
                radius = r,
                time = time,
                laps = laps
            )
            test_ts = 0:time/4:time*laps
            xs_spline, ys_spline, xdots_spline, ydots_spline = LearningWithSimpleModels.eval_all(spl, test_ts)
            @test all([
                isapprox(xs_spline[i+1], xs_spline[i+5]; atol=atol) &&
                isapprox(xs_spline[i+1], xs_spline[i+9]; atol=atol) for i in 0:3
            ])
            @test all([
                isapprox(ys_spline[i+1], ys_spline[i+5]; atol=atol) &&
                isapprox(ys_spline[i+1], ys_spline[i+9]; atol=atol) for i in 0:3
            ])
        end

        let task_time = 10.0
            spl = figure_eight(;
                xdot_0 = nothing,
                ydot_0 = nothing,
                xdot_f = nothing,
                ydot_f = nothing,
                radius = 1.0,
                time = task_time,
                laps = 1
            )
            @test all([
                evaluate(spl, 1.0) == evaluate(spl, 1.0+task_time) == evaluate(spl, 1.0+5*task_time),
                evaluate(spl, 3.5) == evaluate(spl, 3.5+task_time) == evaluate(spl, 3.5+5*task_time)
            ])
        end

    end

    @testset "Model" begin
        let in_size = 10, out_size = 4
            layer_sizes = [in_size, 8, 6, out_size]
            m = LearningWithSimpleModels.make_model(layer_sizes)
            out = @test_nowarn m(ones(in_size))
            @test length(out) == out_size
        end
    end

    @testset "Unicycle Controller" begin
        @test_nowarn test_unicycle_controller(; plot=false)
    end
end
