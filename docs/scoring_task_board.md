# Scoring Algorithm Task Board

This board is the restart point for the scoring workstream. Update status values as:
`todo`, `doing`, `blocked`, or `done`.

| ID | Status | Owner | Task | Restart / Check |
| --- | --- | --- | --- | --- |
| S1 | done | main | Capture scoring requirements from event PDF and current model signals. | `README.md`, event PDF, model overview inspected. |
| S2 | done | subagent | Independently verify scoring schema and metrics from the PDF. | Required CSV/JSON fields, formulas, and ambiguities reported. |
| S3 | done | subagent | Independently verify simulation coordinate systems and available ego signals. | RRHD/Sim3D transform, vehicle signals, and smoke risks reported. |
| S4 | done | main | Implement standalone scoring algorithm. | `scoreNavigationRun` accepts telemetry CSV/table and events JSON/struct. |
| S5 | done | main | Add deterministic scoring tests. | `runScoringTests` passes without launching Simulink. |
| S6 | done | main | Run model structural check. | `model_check` has 3 known warnings: unconnected `Algorithm` outputs. |
| S7 | done | main | Run short simulation or document blocker with exact error. | `sim("ControlVehicle","StopTime","0.1")` passed, reached stop time. |
| S8 | done | main | Integrate findings and final notes. | Final answer lists files changed, tests run, simulation result, restart state. |
| S9 | done | main | Add RRHD lane-deviation calculation from vehicle position. | `testComputeLaneDeviationFromRRHD` passed; RRHD smoke at configured start gives `-0.866819 m`. |
| S10 | done | main | Modify model/logging path to generate scoring logs. | `runScoringSimulation(StopTime="0.1")` writes telemetry CSV, events JSON, and score summary. |
| S11 | done | main | Prevent manual Simulink runs from running forever. | Model `StopTime` is `simulationStopTime_s`; default set in `setup_Simulation.m` to `60 s`. |

## Current Critical Checks

- Do not modify official environment assets (`localFolder`, `.xodr`, `.rrhd`, `.osm`) for scoring.
- Keep scoring independent of the 3D simulator so it is testable from logs alone.
- Use the guideline coordinate convention for logs: `ego_heading_deg` is `0 = North`, clockwise positive, `90 = East`.
- Generate `lateral_deviation_m` with `computeLaneDeviationFromRRHD`; it converts Sim3D to RRHD/map coordinates by default.
- `ControlVehicle.slx` logs `Vehicle_Info_Log` to `logsout`; `runScoringSimulation` converts that into scoring files.
- Manual Simulink runs use `simulationStopTime_s`; change it in `setup_Simulation.m` or the base workspace for longer/shorter runs.
- Signal states are ignored for now: `signal_detected`, `signal_state_detected`, and red-signal compliance are excluded by default.
- Treat task completion as a gate for navigation-time points only; compliance metrics still score from partial logs.
- Existing model warning to account for: `Algorithm` outputs are currently unconnected at the top level.
