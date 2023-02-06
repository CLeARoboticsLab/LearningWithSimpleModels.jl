module LearningWithSimpleModels

include("spline.jl")
export Spline, evaluate, figure_eight

include("dynamics.jl")
export SystemParameters, SimpleDynamics, ActualDynamics

include("train.jl")
export train

end
