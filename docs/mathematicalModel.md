# Mathematical model

## Differential-drive kinematics
Wheel speeds (rad/s) to body velocities, wheel radius r = 0.05 m, track b = 0.32 m:

    v = r/2 (wR + wL)        w = r/b (wR - wL)
    wR = (v + w b/2) / r     wL = (v - w b/2) / r

Pose q = [x y theta]:  x' = v cos(theta), y' = v sin(theta), theta' = w.
Forward Euler (default) or midpoint integration, yaw wrapped to [-pi, pi).
Verification (`testKinematics`): zero, straight, in-place and circular motion against
analytic solutions; midpoint closure error < Euler closure error.

## Wheel motor and speed loop
First-order motor, time constant tau = 0.06 s, exact zero-order-hold discretisation:

    w[k+1] = w[k] + (1 - exp(-dt/tau)) (u[k] - w[k])

PI with feedforward: u = w_ref + Kp e + Ki integral(e) (+ Kd de/dt), Kp = 0.5, Ki = 3,
integral clamped and frozen while the output saturates (anti-windup).
With dt/tau = 0.83 an Euler motor model and Kp = 2 is unstable (loop gain > 2);
the ZOH model and Kp = 0.5 give a loop gain of 0.85.

## A* global planner
Binary grid 0.1 m. Hard inflation radius robot.radius + safety.margin + half a cell
(0.42 m). Soft cost band 0.35 m beyond it (edge cost x (1 + 1.5)) keeps paths near
aisle centres. 8-connected, costs 1 / sqrt(2) cells, octile heuristic
h = max(dx,dy) + (sqrt(2)-1) min(dx,dy) (admissible, consistent), no diagonal corner
cutting, binary-heap open list. Line-of-sight smoothing on the banded grid, then
resampling at 0.1 m.

## Path tracking
Look-ahead point L = 0.6 m ahead along the path:

    e_theta = wrap(atan2(yt - y, xt - x) - theta)
    w = Kp_heading e_theta,  v = vMax max(0, cos e_theta),  v <= 0.8 d_goal + 0.05

## DWA-style local planner
Candidates: 7 x 15 grid of (v, w) reachable within 0.4 s under aMax, alphaMax, plus
the nominal tracking command and a braking command.
Safety rollout (2.5 s, 0.1 s steps): hold (v, w) for 0.5 s, then brake at aMax and
alphaMax to rest (stopping ability is part of the check).
Moving obstacles: constant-velocity prediction p(tau) = p0 + v tau, inflated by
r_obs + 0.05 + 0.10 tau (uncertainty growth).
Reject if static clearance < min(0.05, current clearance - 0.02) or dynamic
clearance < 0.05 at any step. Cost on the constant-command rollout (1.5 s):

    J = wg (d_goal/(vMax T) + 0.3 |heading error|/pi) + wp d_path
        + wc max(0, 0.5 - clearance)/0.5 + wu (dv^2/vMax^2 + dw^2/wMax^2)
        + ws (vMax - v)/vMax

Weights 1.0 / 0.8 / 1.0 / 0.1 / 0.3. If nothing is safe: evasive command maximising
dynamic clearance (static-safe); if nothing is static-safe: controlled stop.

## Perception
LiDAR: 181 beams, 270 deg, 6 m, Gaussian noise 0.02 m, optional missed and
spurious returns; static ray marching at 0.05 m plus analytic ray-circle tests.
Known-map gate: returns within max(0.15, 2.5 sigma) m of the prior map are ignored.
Clustering by beam order (gap 0.30 + 3 sigma m), single returns discarded as noise.
Tracking: greedy nearest neighbour with 0.8 m gate, alpha-beta filter
(alpha 0.35, beta 0.06), merge within 0.45 m, expiry after 1 s, moving when
>= 5 hits and speed > 0.12 m/s (hysteresis).
Observed grid: a cell is occupied after >= 2 returns, cleared by beams passing
through it, expires after 15 s.

## Replanning rule
Blocked if any remaining waypoint within 8 m is closer than radius + margin/2 to an
observed obstacle. Replan when blocked for 2 consecutive steps, every 2 s if the
observed map changed, or after > 1 m deviation; at most every 0.5 s. A new path
replaces the old one only when A* succeeds; if the old path is blocked and no
route exists the robot stops and reports `unreachable` after 3 s at rest.
