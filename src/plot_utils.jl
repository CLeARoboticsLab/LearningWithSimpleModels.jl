function plot_losses(losses)
    fig = Figure()
    ax = Axis(fig[1,1], xlabel="Iteration", ylabel="Loss")
    lines!(ax, losses)
    display(GLMakie.Screen(), fig)
end