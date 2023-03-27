const START_CMD = JSON.json(Dict("action" => "start_experiment")) * "\n"
const STOP_CMD = JSON.json(Dict("action" => "stop_experiment")) * "\n"
const GET_TIME_CMD = JSON.json(Dict("action" => "get_time_elapsed")) * "\n"
const GET_ROLLOUT_DATA = JSON.json(Dict("action" => "get_rollout_data")) * "\n"

struct Connections
    feedback::Connection
    control::Connection
    timing::Connection
    rollout::Connection
end

function open_connections()
    ip = "192.168.1.223"
    feedback_port = 42422
    control_port = 42426
    timing_port = 42423
    rollout_port = 42425

    feedback = open_feedback_connection(ip, feedback_port)
    control = open_connection(ip, control_port)
    timing = open_connection(ip, timing_port)
    rollout = open_connection(ip, rollout_port)

    return Connections(feedback, control, timing, rollout)
end

function close_connections(connections::Connections)
    close_connection(connections.feedback)
    close_connection(connections.control)
    close_connection(connections.timing)
    close_connection(connections.rollout)
end

function start_rollout(connections::Connections)
    send(connections.timing, START_CMD)
end

function stop_rollout(connections::Connections)
    send(connections.timing, STOP_CMD)
end

function time_elapsed(connections::Connections)
    payload = send_receive(connections.timing, GET_TIME_CMD)
    data = JSON.parse(String(payload))
    return data["elapsed_time"]
end

function state(connections::Connections)
    feedback_data = receive_feedback_data(connections.feedback)
    x = feedback_data.position[1]
    y = feedback_data.position[2]

    rot_mat = QuatRotation(
        feedback_data.orientation[1],
        feedback_data.orientation[2],
        feedback_data.orientation[3],
        feedback_data.orientation[4]
    )
    heading = rot_mat*[1,0,0]
    θ = atan(heading[2],heading[1])

    v = norm(feedback_data.linear_vel[1:2])
    if abs(atan(feedback_data.linear_vel[2],feedback_data.linear_vel[1]) - θ) > π/2
        v = -v
    end

    return [x,y,v,θ]
end

function send_command(
    connections::Connections, 
    ctrl_params::ControllerParameters, 
    spline_seg::Spline, 
    gains_adjustment::Vector{Float64}
)
    
    payload = spline_seg.coeffs_x
    append!(payload, spline_seg.coeffs_y)
    append!(payload, [
        ctrl_params.kx,
        ctrl_params.ky,
        ctrl_params.kv,
        ctrl_params.kϕ,
        ctrl_params.ka,
        ctrl_params.kω] + gains_adjustment)
    command = JSON.json(Dict("array" => payload)) * "\n"
    send(connections.control, command)
end

function rollout_data(connections::Connections)
    payload = send_receive(connections.rollout, GET_ROLLOUT_DATA)
    data = JSON.parse(String(payload))
    return (
        seg_idxs = convert(Vector{Int64}, data["seg_idxs"]) .+ 1,
        ts = convert(Vector{Float64}, data["ts"]),
        xs = convert(Matrix{Float64}, reduce(hcat,data["xs"])),
        us = convert(Matrix{Float64}, reduce(hcat,data["us"])),
        ctrl_setpoints = convert(Matrix{Float64}, reduce(hcat,data["ctrl_setpoints"]))
    )
end