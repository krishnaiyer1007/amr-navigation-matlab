# Architecture

## Data flow per control step (dt = 0.05 s)

```
simAdvanceWorld  ground truth: scripted obstacles move, hidden blockers appear
simScan          LiDAR ray casting from the TRUE pose (noise, misses, false returns)
simPerceive      returns -> world points with the ESTIMATED pose
                 known-map gate -> clustering -> alpha-beta tracks (moving / static)
                 -> observed-obstacle grid (evidence counting, ray clearing, expiry)
simReplan        (controller P) path blockage check, debounced A* replanning,
                 keep previous safe path until a valid replacement exists
simCommand       look-ahead heading controller -> local safety layer
                   B0: none   B1: proximity / TTC stop   P: DWA-style sampler
limitCommand     speed and acceleration limits (Safety Supervisor)
plant            inverse kinematics -> wheel PI + feedforward -> 1st-order motors
                 -> forward kinematics -> pose integration (Euler / midpoint)
simPostStep      ground-truth collision + clearance, logging, goal / failure tests
```

`runSimulation` (MATLAB loop) and `simulink/amrNavigation.slx` execute the same
stage functions, so the Simulink model reproduces the script bit-for-bit
(see `scripts/compareSimulink.m`).

## Ground truth vs perception

| Used only by the simulator (truth) | Used by the navigation stack |
|---|---|
| `world.truth` (known map + hidden blockers) | `world.known` (prior map) |
| `world.dyn` (moving obstacle states) | LiDAR returns, tracks, observed grid |
| `robot.pose` | `robot.poseEst` |

No navigation function reads `world.truth`, `world.dyn` or `robot.pose`.

## Folder map

| Folder | Content |
|---|---|
| `config/` | `defaultConfig.m`, every tunable parameter with units |
| `maps/` | `scenarioDefinitions.m` (S01 to S12, seeded variants), `warehouse01.mat` |
| `src/mapping` | map layouts, world initialisation, obstacle motion |
| `src/planning` | inflation, A*, smoothing, blockage check, `planGlobalPath` |
| `src/control` | look-ahead tracking, command limiter, wheel PI |
| `src/dynamics` | differential-drive plant, robot state |
| `src/sensing` | LiDAR model, local observed-obstacle map |
| `src/tracking` | clustering, association, alpha-beta filter |
| `src/avoidance` | DWA-style local planner, B1 proximity stop |
| `src/collision` | footprint collision and clearance (ground truth) |
| `src/core` | simulation stages, `runSimulation` |
| `src/metrics` | metrics, tables, summaries |
| `src/visualization` | map drawing, per-run figures, animation |
| `simulink/` | model builder, bridge, runner |
| `tests/` | unit and integration tests |
| `scripts/` | baseline, full system, benchmark, plots, Simulink comparison |

## Simulink subsystems

| Subsystem | Implementation | Inputs | Outputs |
|---|---|---|---|
| Scenario & World | bridge -> `simAdvanceWorld` | t | obstacle states |
| Range Sensor | bridge -> `simScan` | true pose | ranges |
| State Estimator | native (true pose pass-through) | pose, wheel speeds | pose estimate |
| Local Mapping & Tracking | bridge -> `simPerceive` | ranges, pose, t | track counts |
| Global Planner | bridge -> `simReplan` | t | waypoints, blocked, replans |
| Local Planner | bridge -> `simCommand` | pose, previous command | v, w request, mode |
| Safety Supervisor | native MATLAB Function | request, previous command | limited command, wheel targets |
| Wheel Controller | native MATLAB Function | wheel targets, wheel speeds | motor inputs |
| Differential Drive Plant | native MATLAB Function + Unit Delays | motor inputs | wheel speeds, v, w, pose |
| Logger & Visualizer | bridge -> `simPostStep` + To Workspace | pose, v, w, command | done flag (Stop block) |
