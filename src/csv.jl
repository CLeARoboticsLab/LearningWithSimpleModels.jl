function make_csv(savepath::String, r::RolloutData)
    arr = hcat(r.ts, r.xs', r.us')
    h = [
        "time (sec)",
        "x position (m)",
        "y position (m)",
        "forward velocity (m/s)",
        "heading angle (rad), from x-axis, positive is CCW",
        "throttle (unitless; -1 to 1)",
        "steering angle (unitless; -1 to 1)"
    ]
    CSV.write(savepath,  Tables.table(arr), header=h)
    return arr
end