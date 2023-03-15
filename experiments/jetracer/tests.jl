function test_spline_control()
    ip = "192.168.1.135"   
    time_port = 42423
    spline_port = 42424

    start_cmd = JSON.json(Dict("action" => "start_experiment")) * "\n"
    stop_cmd = JSON.json(Dict("action" => "stop_experiment")) * "\n"

    spl = Spline(;
        xs = [0.0, 2.0],
        ys = [0.0, 2.0],
        ts = [0.0, 10.0]
    )
    coeffs = spl.coeffs_x
    append!(coeffs, spl.coeffs_y)
    spline_coeffs = JSON.json(Dict("array" => coeffs)) * "\n"

    time_connection = open_connection(ip, time_port)
    spline_connection = open_connection(ip, spline_port)

    send(spline_connection, spline_coeffs)
    sleep(.5)
    send(time_connection, start_cmd)
    sleep(12.0)

    send(time_connection, stop_cmd)
    close_connection(time_connection)
    close_connection(spline_connection)
end