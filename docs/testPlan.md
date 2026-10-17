# Test plan

## Unit tests (`tests/`, run `runAllTests`)
| File | Checks |
|---|---|
| testKinematics | zero / straight / in-place / circular motion, wheel mapping, Euler vs midpoint, yaw wrap, limits, wheel PI convergence |
| testAStar | octile optimality, equality with brute-force Dijkstra, no corner cutting, no-path, goal/start blocked, snapping, map conversions, smoothed path validity |
| testCollision | open space, overlap, 1 cm clear, exact touch, square-cell corner distance, dynamic circles |
| testAvoidance | dynamic window limits, free-space progress, wall rejection, moving-obstacle prediction, stop when enclosed, B1 stop |
| testMapping | ray casting geometry, max range / FOV, analytic circles, noise and miss statistics, local map stamping / clearing / expiry, velocity estimation |
| testReplanning | blockage detection, valid detour, no-path status, closed-loop S06 replanning, S07 safe stop |
| testDeterminism | identical seeds give identical trajectories and metrics; RNG isolation |

Integration: `tests/runScenarios.m` checks every scenario outcome with controller P.

## Scenarios
S01 to S12 as in the README (section 12), defined in `maps/scenarioDefinitions.m`.
Seeded variants (seed > 0) randomise obstacle speeds (x0.8 to 1.2), start delays
(+0 to 3 s), blocker appearance times and start / goal (+-0.2 m).

## Benchmark protocol (`scripts/runBenchmark.m`)
30 variants = {S02 S03 S04 S05 S06 S08 S09 S10 S11 S12} x 3 seeds, each run with
B0, B1 and P on the same scenario and noise seed (90 paired runs).
Sensitivity (P only, one variable at a time, 10 variants per level): LiDAR noise,
obstacle speed, robot max speed, safety margin, missed-return probability.

## Failure definitions
collision (footprint overlap with truth), stall (< 0.3 m progress in 25 s, includes
oscillation), unreachable (no valid route, robot stopped), timeout (120 s),
invalid_path (initial plan impossible).

## Calibration log
| Parameter | Start | Final | Reason |
|---|---|---|---|
| wheelPID.Kp | 2.0 | 0.5 | loop unstable with dt/tau = 0.83 |
| track.alpha / beta | 0.5 / 0.3 | 0.35 / 0.06 | velocity noise amplified at 20 Hz caused phantom moving tracks |
| track.minHits | 3 | 5 | same |
| dwa.wPath / wClear | 0.4 / 0.6 | 0.8 / 1.0 | corner cutting gave 0.03 to 0.07 m clearance |
| planner costBand | none | 0.35 m, x1.5 | A* hugged inflation boundary at corners |
| localMap evidence | single return | >= 2 returns in clusters | 1 % false returns blocked paths |
| S12 patrol routes | ended at candidate goals | kept >= 2 m from start/goal points | sensitivity run found unavoidable collisions with non-reactive obstacles parked at the goal (vMax 0.3) |
| obstacle motion | instant reversal at patrol ends | 0.5 m/s^2 accel/decel, 1 s dwell | instant reversals are physically impossible (README 12) and caused head-on collisions |
| evasive fallback | max min-distance | max time to first violation | chose forward motion toward a reversing obstacle |
| planner.replanPeriod | 0.5 s | 2.0 s (map changed only) | avoid replanning every scan |
