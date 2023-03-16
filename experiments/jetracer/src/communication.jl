const START_CMD = JSON.json(Dict("action" => "start_experiment")) * "\n"
const STOP_CMD = JSON.json(Dict("action" => "stop_experiment")) * "\n"
const GET_TIME_CMD = JSON.json(Dict("action" => "get_time_elapsed")) * "\n"

struct Connections
    feedback::Connection
    control::Connection
    timing::Connection
end

function open_connections()
    ip = "192.168.1.135"
    feedback_port = 42422
    control_port = 42424
    timing_port = 42423 

    feedback = open_feedback_connection(ip, feedback_port)
    control = open_connection(ip, control_port)
    timing = open_connection(ip, timing_port)

    return Connections(feedback, control, timing)
end

function close_connections(connections::Connections)
    close_connection(connections.feedback)
    close_connection(connections.control)
    close_connection(connections.timing)
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