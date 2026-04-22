# Evaluation Mapping

## Existing Work Reused

- Dynamics core and simulation style from original `claude_task1.m`:
  - state-space formulation
  - mass matrix, Coriolis, gravity, friction concepts
  - forward kinematics and animation style
- Simulink files copied for reference and continuity.

## Completion in This Attempt

- Added reusable modular code layout (instead of one long script).
- Completed Task-1 scenario coverage and required `x,y,alpha` outputs.
- Implemented Task-2 model-based + servo partitioned control with gain sweep analysis.
- Implemented Task-3 spline trajectory tracking with zero endpoint slope constraints.
- Implemented Task-4 Cartesian disturbance force handling through Jacobian transpose.
- Implemented Task-5 Cartesian controller demonstration.

## Lecture Fit

- Dynamics: `M(q) qdd + C(q,dq) dq + G(q) = tau` and state-space simulation.
- Control partitioning: model-based cancellation + servo error dynamics.
- Jacobian use in both velocity and force domain (`tau = J^T F`).
- Trajectory generation/tracking and Cartesian-space control.
