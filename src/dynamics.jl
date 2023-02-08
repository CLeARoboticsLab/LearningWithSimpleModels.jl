f_simple(dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)
f_actual(dyn::Dynamics, t::Float64, dt::Float64, x::Vector{Float64}, u::Vector{Float64}) = dyn.f(dyn,t,dt,x,u)

function rollout_actual_dynamics(
    task::Spline, 
    model::Chain,
    actual_dynamics::Dynamics, 
    controller::Controller, 
    sim_params::SimulationParameters
)  
    task_time, n_segments, segment_length = properties(task, sim_params)

    t0_segs = zeros(n_segments)

    n_states = length(sim_params.x0)
    x0_segs = zeros(n_states, n_segments)

    xs_actual = zeros(n_states, Integer(round(task_time/sim_params.dt)))
    us_actual = zeros(sim_params.n_inputs, Integer(round(task_time/sim_params.dt)))

    x = sim_params.x0
    for j in 1:n_segments
        # start and end time of this segment
        t0_seg = (j-1)*sim_params.model_dt
        t0_segs[j] = t0_seg
        tf_seg = t0_seg + sim_params.model_dt - sim_params.dt

        x0_segs[:,j] = x

        # setpoint is task evaluated at next model call time (segment end time)
        setpoint = evaluate(task, tf_seg)
        new_setpoint = new_setpoint_from_model(setpoint, model, t0_seg, x, task_time)

        # rollout on this segment
        ts = t0_seg:sim_params.dt:tf_seg
        for (i,t) in enumerate(ts)
            overall_idx = (j-1)*segment_length + i
            xs_actual[:,overall_idx] = x
            u = next_command(controller, x, new_setpoint)
            us_actual[:,overall_idx] = u
            x = f_actual(actual_dynamics, t, sim_params.dt, x, u)
        end
    end
    xf = x
    return t0_segs, x0_segs, xs_actual, us_actual, xf
end
