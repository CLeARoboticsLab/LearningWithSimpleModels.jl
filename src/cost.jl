abstract type CostParameters end

Base.@kwdef struct QuadraticCostParameters <: CostParameters
    Q::Matrix{Float64}
    R::Matrix{Float64}
end

Base.@kwdef struct Cost
    params::CostParameters
    g::Function
end

quadratic_cost(; Q::Matrix{Float64}, R::Matrix{Float64}) = Cost(;
    params = QuadraticCostParameters(;Q=Q, R=R),
    g = (cost::Cost,x::Vector{Float64},u::Vector{Float64}) -> begin
        return x'*cost.params.Q*x + u'*cost.params.R*u
    end
)

stage_cost(cost::Cost,x::Vector{Float64},u::Vector{Float64}) = cost.g(cost::Cost,x,u)
