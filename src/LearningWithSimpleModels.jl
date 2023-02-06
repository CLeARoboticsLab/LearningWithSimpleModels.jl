module LearningWithSimpleModels

include("spline.jl")
export Spline, evaluate, figure_eight

include("dynamics.jl")
export SystemParameters, SimpleDynamics, ActualDynamics

include("controller.jl")
export ControllerParameters, Controller

include("train.jl")
export train

end
