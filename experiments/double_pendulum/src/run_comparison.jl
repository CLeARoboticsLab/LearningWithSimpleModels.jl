function generate_comparison_data()
    batches = 5
    for i in 1:batches
        
        # no mismatch
        base_path = ".data/results_pend/no_mismatch"

        # # closed loop
        @info "Running closed loop, no mismatch, $i/$batches"
        save_path = joinpath(base_path, "closed_loop_0$i")
        train(
            dp_simple_dynamics(; mass_mismatch=1.0),
            dp_actual_dynamics();
            controller = dp_controller(),
            cost = dp_cost(),
            task = dp_task(),
            algo = dp_training_algorithm(),
            training_params = dp_training_parameters(; save_path=save_path),
            sim_params = dp_simulation_parameters()
        )

        # open loop
        @info "Running open loop, no mismatch, $i/$batches"
        save_path = joinpath(base_path, "open_loop_0$i")
        train(
            dp_simple_dynamics_open_loop(; mass_mismatch=.99),
            dp_actual_dynamics_open_loop();
            controller = dp_controller_open_loop(),
            cost = dp_cost_open_loop(),
            task = dp_task_open_loop(),
            algo = dp_training_algorithm_open_loop(),
            training_params = dp_training_parameters_open_loop(; save_path=save_path),
            sim_params = dp_simulation_parameters_open_loop()
        )

        # mismatch
        mass_mismatch = 1.5
        base_path = ".data/results_pend/mismatch"

        # closed loop
        @info "Running closed loop, mismatch, $i/$batches"
        save_path = joinpath(base_path, "closed_loop_0$i")
        train(
            dp_simple_dynamics(; mass_mismatch=mass_mismatch),
            dp_actual_dynamics();
            controller = dp_controller(),
            cost = dp_cost(),
            task = dp_task(),
            algo = dp_training_algorithm(),
            training_params = dp_training_parameters(; save_path=save_path),
            sim_params = dp_simulation_parameters()
        )

        # open loop
        @info "Running open loop, mismatch, $i/$batches"
        save_path = joinpath(base_path, "open_loop_0$i")
        train(
            dp_simple_dynamics_open_loop(; mass_mismatch=mass_mismatch),
            dp_actual_dynamics_open_loop();
            controller = dp_controller_open_loop(),
            cost = dp_cost_open_loop(),
            task = dp_task_open_loop(),
            algo = dp_training_algorithm_open_loop(),
            training_params = dp_training_parameters_open_loop(; save_path=save_path),
            sim_params = dp_simulation_parameters_open_loop()
        )
    end
end