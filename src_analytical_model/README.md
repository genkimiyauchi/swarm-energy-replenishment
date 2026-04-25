# Analytical model

This repository contains the code for the analytical model and derived predicitions and comparisons presented in the paper
"Optimal Performance of Cooperative Energy-Replenishment in Robot Swarms" that was submitted to DARS 2026.

## Installing Python dependencies

To run the scripts in this directory, install the Python dependencies in requirements.txt using your choice of virtual environment. Here, we assume using venv:

```bash
python3 -m venv venv_analytical_model
source venv_analytical_model/bin/activate
pip install -r requirements.txt
```

## Usage
Below is an overview of the scripts and their functionality.

### Core functions
The script `calculate_functions.py` provides implementations of:
- the **Cooperative Strategy (CS)** and **Individual Strategy (IS)** described in the paper,
- functions for plotting the **feasible region**,
- utilities for comparing analytical results with simulation data.

Example usage is demonstrated in the scripts below.

### Comparing the model to simulation results
To compare analytical results with simulation data, run:

```bash
python3 analytical_vs_simulations.py
```

This script:
- computes and visualizes deviations between the analytical model and simulation runs,
- plots feasible regions for the specified configurations.

To use custom simulation data, update the log file paths in the plotting function calls.

### Comparing the model to the optimizers
To compare the analytical model with the optimizers **SciPy (`trust-constr`)** and **Ipopt (via CasADi)**, run:

```bash
python3 analytical_vs_optimizers.py
```

This script reports relative and absolute statistics for **computation time**, **duty cycle**, and **energy efficiency**.

Usage notes:
- Set `OPTIMIZE = False` to use the provided result files.
- Set `OPTIMIZE = True` to run the optimizers and save results as `.npy` files.

### Running model predictions

To reproduce the model predictions and plots presented in the paper, run:

```bash
python3 calculate_for_capbot_epuck_values.py
```

---