function gradient_estimate(
    r::RolloutData,
    task::Spline,
    model::Chain,
    simple_dynamics::Dynamics,
    controller::Controller,
    cost::Cost,
    sim_params::SimulationParameters
)
    task_time = end_time(task)

    loss, grads = withgradient(model) do model
        loss = 0.0
        n_segments = length(r.t0_segs)
        prev_setpoint = evaluate(task, r.t0_segs[1])
        for j in 1:n_segments-1
            t0_seg = r.t0_segs[j]
            tf_seg = t0_seg + sim_params.model_dt
            x = r.x0_segs[:,j]
            
            setpoint = evaluate(task, tf_seg)
            new_setpoint, gains_adjustment = call_model(sim_params, setpoint, 
                                                        model, t0_seg, x, task_time)
            t = t0_seg - r.task_t0
            spline_seg = spline_segment(t, t + sim_params.model_dt, prev_setpoint, new_setpoint)
            prev_setpoint = new_setpoint

            idx_seg = r.idx_segs[j]
            x = r.xs[:,idx_seg]
            for i in r.idx_segs[j]:r.idx_segs[j+1]-1
                t = r.ts[i]
                ctrl_setpoint = evaluate(spline_seg, t; wrap_time=false)
                u = next_command(controller, x, ctrl_setpoint, gains_adjustment)
                loss = loss + stage_cost(cost, x, evaluate(task, r.task_t0 + t), u)
                x_actual_next = r.xs[:,i+1]
                x = (
                    f_simple(simple_dynamics, t, sim_params.dt, x, u)
                    - f_simple(simple_dynamics, t, sim_params.dt, r.xs[:,i], r.us[:,i])
                    + x_actual_next
                )
                if !isapprox(x, x_actual_next)
                    println("Mismatched x at index $(i), t: $(t)
                    udiff: $(u - r.us[:,i])
                    u: $(u)
                    r.u: $(r.us[:,i])
                    xdiff: $(x - x_actual_next)")
                end
            end
        end
        return loss
    end
    return loss, grads
end