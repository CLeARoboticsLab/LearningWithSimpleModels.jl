quadratic_cost(; Q::Matrix{Float64}, R::Matrix{Float64}) = Cost(;
    params = QuadraticCostParameters(;Q=Q, R=R),
    g = (cost::Cost,x::Vector{Float64},u::Vector{Float64}) -> begin
        return x'*cost.params.Q*x + u'*cost.params.R*u
    end
)

stage_cost(cost::Cost, t::Float64, x::Vector{Float64}, x_des::Vector{Float64}, 
            task::AbstractTask, u::Vector{Float64}) = cost.g(cost::Cost,t,x,x_des,task,u)
