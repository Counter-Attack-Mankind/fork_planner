# NLP latency audit

This tool measures the deadlock-to-motion pipeline without publishing to any
planner, coordination, vehicle-control, or recovery-state topic. The live
collector only subscribes to:

- `/rosout_agg`
- `/forklift_planner/recovery_snapshot`
- `/forklift_planner/nlp_recovery_trajectory`

It can also poll the planner diagnostic directory read-only. Each record keeps
wall-clock (`time.time`), monotonic-clock (`time.monotonic`), ROS time, message
header time, or simulation time as separate fields. Missing boundaries are
reported as `UNKNOWN`.

## Reproduce four vehicles, seed 2022

From WSL Ubuntu 20.04 with ROS Noetic:

```bash
cd /home/lsj/fork_planner_phase1_ws/src/fork_planner
tools/nlp_latency_audit/run_audit.sh reproduce 900
```

The diagnostic launch loads the normal project YAML, overrides only the two
seed parameters, vehicle count, output paths, and `execute_after_solve=true`,
and does not start RViz. It does not alter any project source/configuration.
The run stops after the first participant motion or after the timeout.

Output is written below `tools/nlp_latency_audit/runs/<UTC>_seed2022/`:

- `audit.jsonl`: raw monotonic/ROS/topic observations
- `console.log`: roslaunch console evidence
- `diagnostics/`: normal planner diagnostics for this run
- `results/`: NLP input, initial guess, optimized trajectory, solver report
- `latency_report.md`: stage table and evidence classification

Set `FORK_PLANNER_WS` if the Catkin workspace differs. Set
`NLP_AUDIT_OUTPUT=/absolute/path` to place run artifacts elsewhere.

## Attach to an already running system

```bash
tools/nlp_latency_audit/run_audit.sh collect /tmp/audit.jsonl 900
```

The command is read-only. It detects first motion from a participant pose delta
or non-zero speed in RecoverySnapshot after the trajectory topic is observed.

## Analyze existing evidence

```bash
python3 tools/nlp_latency_audit/audit_latency.py analyze \
  --events /tmp/audit.jsonl \
  --ros-log ~/.ros/log/latest/rosout.log \
  --diagnostic-log /path/to/deadlock_000.log \
  --solver-report /path/to/solver_report.yaml \
  --output /tmp/latency_report.md
```

`solver_report.wall_time` is shown as the CasADi/IPOPT numeric-call time only.
It is not presented as the duration of the whole `NlpSolver::solve()` call.
