# Final report

The full technical report follows the outline below.


1. Abstract and objectives. Research question from README section 1.
2. Industrial problem and motivation: shared warehouse aisles, pedestrians and forklifts.
3. Related work: A* (Hart, Nilsson, Raphael 1968), DWA (Fox, Burgard, Thrun 1997),
   velocity obstacles, pure pursuit, occupancy grids (Elfes 1989), alpha-beta tracking.
4. Requirements and architecture: `docs/architecture.md`, Simulink screenshot (fig 10).
5. Modelling and controller design: `docs/mathematicalModel.md`.
6. Implementation: MATLAB stage functions, Simulink model, configuration table.
7. Test plan: `docs/testPlan.md`, unit test report in `results/logs/`.
8. Results: `results/tables/benchmark_summary.csv`, figures 7, 8, 8b, 9;
   representative runs `rep_S05_P_*`, `rep_S05_B0_*`, `rep_S06_P_*`, `rep_S11_P_*`.
9. Discussion: trade-offs (time vs safety), failure analysis from paired outcomes,
   computational load (step and planning times in the tables), limitations
   (perfect localisation, circular obstacles, constant-velocity prediction,
   2D only, scripted obstacles that do not react to the robot).
10. Conclusions and further work: README section 21.
11. Appendices: configuration (`config/defaultConfig.m`, config hash in logs),
    extra plots, test results, MATLAB vs Simulink comparison table.
