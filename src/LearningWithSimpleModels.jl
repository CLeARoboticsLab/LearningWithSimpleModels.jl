module LearningWithSimpleModels

using Flux:
    Dense,
    Chain,
    zeros32,
    setup,
    Adam
import ProgressMeter

include("types.jl")
export Spline, 
    SystemParameters, SimpleDynamics, ActualDynamics,
    ControllerParameters, Controller,
    CostParameters, QuadraticCostParameters, Cost,
    TrainingParameters

include("spline.jl")
export evaluate, figure_eight

include("dynamics.jl")
include("controller.jl") 

include("cost.jl")
export quadratic_cost

include("model.jl")

include("train.jl")
export train

end
