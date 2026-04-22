# Josh-attempt (ESE446 Final Project)

This folder is a full MATLAB-based completion attempt for Tasks 1-5 of the ESE446 final project, built from useful code in `reference-original`.

## Structure

- `reference-original/`
  - Original files copied for traceability (`claude_task1.m`, `simple-animate.m`, `model_task1.slx`, `model_task2.slx`)
- `src/`
  - Reusable robot model, kinematics, Jacobian, dynamics, simulator, and controller helpers
- `tasks/`
  - `run_task1.m` ... `run_task5.m`
  - `run_all_tasks.m`
- `plots/`
  - Output figures and MAT logs after running tasks

## Quick Start

1. Open MATLAB in `e:\4460\Josh-attempt\tasks`.
2. Run either:
   - `run_all_tasks`
   - or any single task: `run_task1`, `run_task2`, ..., `run_task5`
3. Check outputs in `e:\4460\Josh-attempt\plots\taskX`.

## What Each Task Script Does

- `run_task1`
  - Runs all required Task-1 cases:
    - no torque/no gravity
    - no torque/with gravity
    - tau1 equilibrium compensation only
    - tau1+tau2 equilibrium compensation
    - tau1+tau2+tau3 equilibrium compensation
  - Saves joint + end-effector (`x,y,alpha`) plots per case.

- `run_task2`
  - Implements control-law partitioning (model-based + servo) using computed-torque PD structure.
  - Applies step commands one joint at a time.
  - Sweeps `Kp = [1,10,100]`, computes rise-time and overshoot summary.

- `run_task3`
  - Builds clamped splines (zero endpoint slopes) from assignment timeline.
  - Tracks trajectory with model-based controller.

- `run_task4`
  - Applies brief Cartesian disturbance impulses (`Fx`, `Fy`, `Malpha`).
  - Maps to joint disturbance torque via `tau_ext = J^T F`.

- `run_task5`
  - Implements Cartesian controller using Jacobian and model-based torque command.
  - Demonstrates independent command moves in `x`, `y`, and `alpha`.

## Notes

- This implementation is MATLAB-first and does not require editing Simulink to finish all tasks.
- Existing Simulink models are preserved in `reference-original` as evidence/base assets.
- If your instructor requires strict Simulink-only block architecture for Tasks 2-5, use this code as validated math/control logic and mirror each controller in your models.
