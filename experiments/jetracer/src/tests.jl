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
    gains = [0.5,0.5,0.5,0.5]
    coeffs_gains = spl.coeffs_x
    append!(coeffs_gains, spl.coeffs_y)
    append!(coeffs_gains, gains)
    display(coeffs_gains)
    spline_gains = JSON.json(Dict("array" => coeffs_gains)) * "\n"

    time_connection = open_connection(ip, time_port)
    spline_connection = open_connection(ip, spline_port)

    send(spline_connection, spline_gains)
    sleep(.5)
    send(time_connection, start_cmd)
    sleep(12.0)

    send(time_connection, stop_cmd)
    close_connection(time_connection)
    close_connection(spline_connection)
end

function test_do_and_wait()
    for i in 1:20
        do_and_wait(0.5) do 
            println(i)
        end
    end
end