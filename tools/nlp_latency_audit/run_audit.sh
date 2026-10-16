#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${FORK_PLANNER_WS:-/home/lsj/fork_planner_phase1_ws}"
OUTPUT_ROOT="${NLP_AUDIT_OUTPUT:-${SCRIPT_DIR}/runs}"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/devel/setup.bash"
set -u

usage() {
  echo "usage: $0 collect OUTPUT.jsonl [timeout_seconds]"
  echo "       $0 analyze OUTPUT.md [audit.jsonl] [solver_report.yaml]"
  echo "       $0 reproduce [timeout_seconds]"
}

command_name="${1:-}"
case "${command_name}" in
  collect)
    output="${2:?missing OUTPUT.jsonl}"
    timeout_seconds="${3:-900}"
    exec python3 "${SCRIPT_DIR}/audit_latency.py" collect \
      --output "${output}" --timeout "${timeout_seconds}" --stop-after-motion
    ;;
  analyze)
    output="${2:?missing OUTPUT.md}"
    events="${3:-}"
    report="${4:-}"
    args=(analyze --output "${output}")
    [[ -n "${events}" ]] && args+=(--events "${events}")
    [[ -n "${report}" ]] && args+=(--solver-report "${report}")
    exec python3 "${SCRIPT_DIR}/audit_latency.py" "${args[@]}"
    ;;
  reproduce)
    timeout_seconds="${2:-900}"
    run_dir="${OUTPUT_ROOT}/$(date -u +%Y%m%dT%H%M%SZ)_seed2022"
    mkdir -p "${run_dir}"
    launch_pid=""
    collector_pid=""
    cleanup() {
      [[ -z "${collector_pid}" ]] || kill "${collector_pid}" 2>/dev/null || true
      [[ -z "${launch_pid}" ]] || kill "${launch_pid}" 2>/dev/null || true
      wait 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM
    roslaunch "${SCRIPT_DIR}/diagnostic_seed2022.launch" \
      output_dir:="${run_dir}" >"${run_dir}/console.log" 2>&1 &
    launch_pid=$!
    for _ in $(seq 1 100); do
      rosparam get /use_sim_time >/dev/null 2>&1 && break
      sleep 0.1
    done
    python3 "${SCRIPT_DIR}/audit_latency.py" collect \
      --output "${run_dir}/audit.jsonl" --timeout "${timeout_seconds}" \
      --stop-after-motion --diagnostics-root "${run_dir}/diagnostics" &
    collector_pid=$!
    wait "${collector_pid}" || true
    collector_pid=""
    kill "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
    launch_pid=""
    solver_report="$(find "${run_dir}/results" -name solver_report.yaml -type f 2>/dev/null | sort | tail -n 1 || true)"
    mapfile -t diagnostic_logs < <(find "${run_dir}/diagnostics" -type f -name 'deadlock*.log' 2>/dev/null | sort || true)
    args=(analyze --events "${run_dir}/audit.jsonl" --ros-log "${run_dir}/console.log"
          --output "${run_dir}/latency_report.md")
    [[ -z "${solver_report}" ]] || args+=(--solver-report "${solver_report}")
    for item in "${diagnostic_logs[@]}"; do args+=(--diagnostic-log "${item}"); done
    python3 "${SCRIPT_DIR}/audit_latency.py" "${args[@]}"
    echo "run directory: ${run_dir}"
    ;;
  *)
    usage
    exit 2
    ;;
esac
