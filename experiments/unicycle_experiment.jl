using LearningWithSimpleModels

struct UnicycleParameters <: SystemParameters end

f(dyn::SimpleDynamics, t, x, u) = x + x*dyn.dt #TODO fix

unicycle_simple_dynamics() = SimpleDynamics(;
    params = UnicycleParameters(),
    f = f, #TODO inline func
    dt = 0.01
)

function run()
    train(unicycle_simple_dynamics())
end
run()