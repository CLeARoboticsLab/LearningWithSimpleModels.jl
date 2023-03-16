function rollout_actual_dynamics(
    connections::Connections,
    model::Chain
)



end

# Executes the function f and waits for the specified delay. The timer starts
# before f is called
function do_and_wait(f, delay)
    cond = Condition()
    Timer(x->notify(cond), delay)
    data = @async $f()
    wait(cond)
    return fetch(data)
end