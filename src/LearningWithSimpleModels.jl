module LearningWithSimpleModels

using Flux:
    Dense,
    Chain,
    zeros32,
    setup,
    Adam
import ProgressMeter

include("spline.jl")
export Spline, evaluate, figure_eight

include("dynamics.jl")
export SystemParameters, SimpleDynamics, ActualDynamics

include("controller.jl")
export ControllerParameters, Controller

include("cost.jl")
export CostParameters, QuadraticCostParameters, Cost, quadratic_cost

include("model.jl")

include("train.jl")
export TrainingParameters, train

end
