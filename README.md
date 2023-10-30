# LearningWithSimpleModels

[![LearningWithSimpleModels](https://github.com/CLeARoboticsLab/LearningWithSimpleModels.jl/actions/workflows/test.yml/badge.svg)](https://github.com/CLeARoboticsLab/LearningWithSimpleModels.jl/actions/workflows/test.yml)

Reference implementation and some experiments from our paper ["Enabling Efficient, Reliable Real-World Reinforcement Learning with Approximate Physics-Based Models"](https://arxiv.org/abs/2307.08168).

```bibtex
@inproceedings{westenbroek2023feedback,
  title={Enabling Efficient, Reliable Real-World Reinforcement Learning with Approximate Physics-Based Models},
  author={Westenbroek, Tyler and Levy, Jacob and Fridovich-Keil, David},
  booktitle={7th Annual Conference on Robot Learning},
  year={2023}
}
```

## Installation

First, clone the repository and start Julia.

```bash
git clone https://github.com/CLeARoboticsLab/LearningWithSimpleModels.jl.git
cd LearningWithSimpleModels.jl
julia
```

Next, activate the project and install the dependencies.

```julia
using Pkg
Pkg.activate(".")
Pkg.instantiate()
```

## Experiments

### Car, Simulated

To run a simulated version of the car experiment, run the below commands from the Julia REPL. The results can be found in the `.data` directory.

```julia
include("experiments/unicycle/unicycle_experiment.jl")
train_unicycle_experiment(); evaluate_unicycle_experiment()
```

To run the car mismatch study found in the appendix, run the below commands from the Julia REPL. The results can be found in the `.data/results/mismatch_study/` directory.

```julia
include("experiments/unicycle/unicycle_experiment.jl")
include("experiments/unicycle/mismatch_study.jl")
mismatch_study()
plot_mismatch_study()
mismatch_car_plots()
```

### Quadruped, Simulated

To run a simulated version of the quadruped experiment (where the quadruped is only modeled as a unicycle), run the below commands from the Julia REPL. The results can be found in the `.data` directory.

```julia
using Pkg
Pkg.activate(".")
using LearningWithSimpleModels
push!(LOAD_PATH, joinpath(pwd(), "experiments/quadruped_simulated/src"))
Pkg.activate("experiments/quadruped_simulated/")
Pkg.develop(path=".")
Pkg.instantiate()
using QuadrupedSimulatedExperiment
QuadrupedSimulatedExperiment.run()
```

### Double Pendulum

To run the Double Pendulum experiment found in the appendix, start by importing the subpackage by running the below commands from the Julia REPL.

```julia
using Pkg
Pkg.activate(".")
using LearningWithSimpleModels
push!(LOAD_PATH, joinpath(pwd(), "experiments/double_pendulum/src"))
Pkg.activate("experiments/double_pendulum/")
Pkg.develop(path=".")
Pkg.instantiate()
using DoublePendulumExperiment
```

To perform a single training run, for both the closed-loop and open-loop cases, run the below. The results can be found in the `.data` directory.

```julia
DoublePendulumExperiment.run()
DoublePendulumExperiment.run_open_loop()
```

To perform training over 64 random seeds, use the following command. The results can be found in the `.data/results_pend/` directory.

```julia
DoublePendulumExperiment.generate_comparison_data()
DoublePendulumExperiment.final_variances_plot()
DoublePendulumExperiment.final_variances_plot_mismatch()
```

