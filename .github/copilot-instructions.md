# Copilot Instructions for matUAVs Project

## Project Overview
The `matUAVs` project focuses on optimizing UAV routing and coverage problems using Mixed Integer Linear Programming (MILP) and heuristic methods. The codebase integrates Gurobi for optimization and custom algorithms for perturbation and solution improvement.

## Key Components
- **src/**: Contains the main source code files.
  - `Solution.cpp`: Implements MILP-based optimization and solution manipulation.
  - `Graph.cpp`: Handles graph-related operations, such as edge insertion and cost calculation.
  - `Input.cpp`: Reads and parses input files defining robots, targets, and depots.
  - `Robot.cpp`: Manages robot properties and configurations.
- **etc/**: Contains auxiliary files and older versions of algorithms.
- **bin/**: Compiled binaries and object files.
- **output/**: Stores results and logs of optimization runs.

## Developer Workflows
### Build
Run the build task defined in `.vscode/tasks.json`:
```cmd
chcp 65001 > nul && .vscode\build.bat
```
This compiles the project and generates the executable in the `bin/` directory.

### Run
Execute the main program from the terminal:
```cmd
bin\main.exe input.txt
```
Replace `input.txt` with the desired input file.

### Debugging
Use the `try-catch` blocks in `Solution.cpp` to handle Gurobi exceptions. For debugging optimization models, print intermediate variables and constraints.

## Project-Specific Conventions
- **MILP Warm Start**: Use `MILP_Warm_Start` to initialize the solver with a feasible solution.
- **Fuel Constraints**: Ensure robots have sufficient fuel for each route segment.
- **Depot Management**: Depots are indexed from `0` to `D-1`, excluding the base depot.
- **Perturbation**: Apply `perturbation()` to explore alternative solutions.

## Integration Points
- **Gurobi**: Integrated for solving MILP models. Key parameters include `GRB_DoubleAttr_Start` and `GRB_DoubleAttr_VarHintVal`.
- **Input Files**: Define robots, targets, depots, and configurations. Example format:
  ```txt
  @base
  Base_1@2337.5 -50

  @target
  425 0
  425 15923.27
  ```

## Examples
### Adding Constraints
In `Solution.cpp`, constraints are added using Gurobi's `addConstr()` method:
```cpp
model.addConstr(rest2_1 == rest2_2, "Rest2_d_" + itos(d));
```

### Fuel Validation
Ensure fuel sufficiency:
```cpp
if (fuel_required > input.getRobotFuel(robotID)) {
    path_temp.pCost = -1;
    return path_temp;
}
```

## Notes
- Follow the naming conventions for variables and methods (e.g., `vars_x`, `vars_d`).
- Use `chrono` for measuring execution time.
- Store results in the `output/` directory for analysis.

Feel free to update this document as the project evolves.
