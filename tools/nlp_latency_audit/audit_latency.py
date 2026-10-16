#!/usr/bin/env python3
"""Read-only latency audit for the deadlock-to-NLP execution pipeline.

The collector subscribes only.  The analyzer deliberately leaves stages UNKNOWN
when the existing program has no externally visible boundary.
"""

import argparse
import glob
import json
import math
import os
import re
import sys
import threading
import time
from datetime import datetime, timezone


UNKNOWN = "UNKNOWN"


EVENT_PATTERNS = (
    ("deadlock_candidate", re.compile(r"\[DEADLOCK\]\s+event=CANDIDATE\b")),
    ("deadlock_confirmed", re.compile(r"\[DEADLOCK\]\s+event=CONFIRMED\b")),
    ("nlp_prep", re.compile(r"\[DEADLOCK\]\s+event=NLP_PREP\b")),
    ("global_freeze", re.compile(r"\[NLP_RECOVERY_FREEZE\]")),
    ("initial_guess_summary", re.compile(r"\[nlp_recovery\]\[initial_guess\]\s+vehicles=")),
    ("initial_guess_saved", re.compile(r"\[nlp_recovery\]\[initial_guess\]\s+saved\b")),
    ("solver_complete", re.compile(r"\[nlp_recovery\]\[solver\]\s+case=")),
    ("result_saved", re.compile(r"\[nlp_recovery\]\[solver\]\s+saved optimized_trajectory")),
    ("trajectory_submitted", re.compile(r"\[nlp_recovery\]\[auto\]\s+submitted\b")),
    ("trajectory_received", re.compile(r"\[NLP_EXEC\]\s+recovery_id=")),
    ("execution_progress", re.compile(r"\[NLP_EXEC_PROGRESS\]")),
    ("execution_finished", re.compile(r"\[NLP_FINISH\]")),
)


def now_record(event, **fields):
    record = {
        "event": event,
        "wall_epoch": time.time(),
        "monotonic": time.monotonic(),
        "utc": datetime.now(timezone.utc).isoformat(),
    }
    record.update(fields)
    return record


def append_jsonl(path, record, lock=None):
    if lock:
        lock.acquire()
    try:
        with open(path, "a", encoding="utf-8") as out:
            out.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
            out.flush()
    finally:
        if lock:
            lock.release()


def extract_recovery_id(text):
    match = re.search(r"recovery_id=(\d+)", text)
    return int(match.group(1)) if match else None


def classify_message(text):
    for event, pattern in EVENT_PATTERNS:
        if pattern.search(text):
            return event
    return None


def parse_sim_time(line):
    match = re.match(r"\[(?:(\d+)min)?(\d+(?:\.\d+)?)s\]", line)
    if match:
        return 60.0 * float(match.group(1) or 0) + float(match.group(2))
    match = re.match(r"\[(\d+(?:\.\d+)?)\]", line)
    return float(match.group(1)) if match else None


class DiagnosticPoller(threading.Thread):
    """Observe flushed diagnostic records without writing to planner topics."""

    def __init__(self, root, output, stop_event, lock):
        super().__init__(daemon=True)
        self.root = root
        self.output = output
        self.stop_event = stop_event
        self.lock = lock
        self.seen = set()

    def run(self):
        while not self.stop_event.wait(0.05):
            # The incident archive may copy old history at incident start.  Use
            # only the live deadlock stream so observation time stays bounded
            # by its normal flush interval instead of being mistaken for the
            # original event time.
            patterns = ("**/deadlock*.log",)
            paths = []
            for pattern in patterns:
                paths.extend(glob.glob(os.path.join(self.root, pattern), recursive=True))
            for path in sorted(set(paths)):
                try:
                    with open(path, "r", encoding="utf-8", errors="replace") as src:
                        lines = src.readlines()
                except OSError:
                    continue
                for line in lines:
                    line = line.strip()
                    event = classify_message(line)
                    if not event:
                        continue
                    key = (path, line)
                    if key in self.seen:
                        continue
                    self.seen.add(key)
                    append_jsonl(self.output, now_record(
                        event, source="diagnostic_file_observation",
                        sim_time=parse_sim_time(line), text=line, file=path,
                        accuracy="bounded_by_diagnostic_flush"), self.lock)


def collect(args):
    try:
        import rospy
        from rosgraph_msgs.msg import Log
        from sandbox_msgs.msg import RecoverySnapshot, NlpRecoveryTrajectory
    except ImportError as error:
        print("ROS Python modules unavailable: {}".format(error), file=sys.stderr)
        return 2

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    open(args.output, "w", encoding="utf-8").close()
    lock = threading.Lock()
    stop_event = threading.Event()
    state = {
        "prep": {},
        "trajectory": {},
        "moved": set(),
    }

    def write(event, **fields):
        append_jsonl(args.output, now_record(event, **fields), lock)

    def snapshot_callback(message):
        rid = int(message.recovery_id)
        if message.recovery_phase != RecoverySnapshot.PHASE_NLP_PREP or rid == 0:
            return
        if rid not in state["prep"]:
            baseline = {}
            for vehicle in message.vehicles:
                if vehicle.id in message.nlp_vehicle_ids and vehicle.pose_valid:
                    baseline[int(vehicle.id)] = (
                        float(vehicle.pose.x), float(vehicle.pose.y),
                        float(vehicle.pose.theta), float(vehicle.current_speed))
            state["prep"][rid] = baseline
            write("snapshot_received", source="recovery_snapshot_topic",
                  recovery_id=rid, ros_receive=rospy.Time.now().to_sec(),
                  header_stamp=message.header.stamp.to_sec(),
                  participants=[int(item) for item in message.nlp_vehicle_ids],
                  reason=message.reason, poses=baseline, accuracy="direct")
            return
        if rid not in state["trajectory"] or rid in state["moved"]:
            return
        baseline = state["prep"].get(rid, {})
        movement = []
        for vehicle in message.vehicles:
            if int(vehicle.id) not in baseline or not vehicle.pose_valid:
                continue
            x0, y0, yaw0, _ = baseline[int(vehicle.id)]
            distance = math.hypot(float(vehicle.pose.x) - x0, float(vehicle.pose.y) - y0)
            speed = abs(float(vehicle.current_speed))
            if distance > args.position_epsilon or speed > args.speed_epsilon:
                movement.append({"id": int(vehicle.id), "distance": distance, "speed": speed})
        if movement:
            state["moved"].add(rid)
            write("first_motion", source="recovery_snapshot_topic",
                  recovery_id=rid, ros_receive=rospy.Time.now().to_sec(),
                  header_stamp=message.header.stamp.to_sec(), movement=movement,
                  accuracy="direct_at_snapshot_frequency")
            if args.stop_after_motion:
                stop_event.set()

    def trajectory_callback(message):
        rid = int(message.recovery_id)
        state["trajectory"][rid] = time.monotonic()
        duration = None
        if message.vehicles and message.vehicles[0].points:
            duration = float(message.vehicles[0].points[-1].time)
        write("trajectory_topic_received", source="nlp_recovery_trajectory_topic",
              recovery_id=rid, ros_receive=rospy.Time.now().to_sec(),
              header_stamp=message.header.stamp.to_sec(), duration=duration,
              vehicle_count=len(message.vehicles), accuracy="direct")

    def rosout_callback(message):
        event = classify_message(message.msg)
        if not event:
            return
        write(event, source="rosout_agg", ros_time=message.header.stamp.to_sec(),
              node=message.name, recovery_id=extract_recovery_id(message.msg),
              text=message.msg, accuracy="direct_ros_log_callback")

    rospy.init_node("nlp_latency_audit", anonymous=True, disable_signals=True)
    rospy.Subscriber("/forklift_planner/recovery_snapshot", RecoverySnapshot,
                     snapshot_callback, queue_size=20)
    rospy.Subscriber("/forklift_planner/nlp_recovery_trajectory",
                     NlpRecoveryTrajectory, trajectory_callback, queue_size=10)
    rospy.Subscriber("/rosout_agg", Log, rosout_callback, queue_size=1000)
    poller = None
    if args.diagnostics_root:
        poller = DiagnosticPoller(args.diagnostics_root, args.output, stop_event, lock)
        poller.start()
    write("collector_started", source="audit", accuracy="direct",
          diagnostics_root=args.diagnostics_root)
    deadline = time.monotonic() + args.timeout if args.timeout > 0 else None
    try:
        while not rospy.is_shutdown() and not stop_event.wait(0.05):
            if deadline is not None and time.monotonic() >= deadline:
                write("collector_timeout", source="audit", accuracy="direct")
                break
    finally:
        stop_event.set()
        if poller:
            poller.join(timeout=1.0)
        write("collector_stopped", source="audit", accuracy="direct")
    return 0


def parse_ros_log(path):
    events = []
    try:
        src = open(path, "r", encoding="utf-8", errors="replace")
    except OSError as error:
        print("warning: cannot read {}: {}".format(path, error), file=sys.stderr)
        return events
    with src:
        for number, raw in enumerate(src, 1):
            text = raw.rstrip("\n")
            ipopt_match = re.search(
                r"Total seconds in IPOPT\s*=\s*([0-9]+(?:\.[0-9]+)?)", text)
            if ipopt_match:
                events.append({
                    "event": "ipopt_reported_seconds",
                    "duration": float(ipopt_match.group(1)), "text": text,
                    "source": "ros_log:{}:{}".format(path, number),
                    "accuracy": "direct_ipopt_console_summary",
                })
                continue
            event = classify_message(text)
            if not event:
                continue
            stamp = None
            match = re.match(r"^(\d{9,}(?:\.\d+)?)\s+", text)
            if match:
                stamp = float(match.group(1))
            else:
                match = re.search(r"\[(\d{9,}(?:\.\d+)?)\]", text)
                if match:
                    stamp = float(match.group(1))
            events.append({
                "event": event, "wall_epoch": stamp, "ros_time": stamp,
                "recovery_id": extract_recovery_id(text), "text": text,
                "source": "ros_log:{}:{}".format(path, number),
                "accuracy": "ros_time_use_sim_time_must_be_checked",
            })
    return events


def parse_diagnostic_log(path):
    events = []
    try:
        src = open(path, "r", encoding="utf-8", errors="replace")
    except OSError as error:
        print("warning: cannot read {}: {}".format(path, error), file=sys.stderr)
        return events
    with src:
        for number, raw in enumerate(src, 1):
            text = raw.rstrip("\n")
            event = classify_message(text)
            if event:
                events.append({
                    "event": event, "sim_time": parse_sim_time(text),
                    "recovery_id": extract_recovery_id(text), "text": text,
                    "source": "diagnostic_log:{}:{}".format(path, number),
                    "accuracy": "direct_sim_time_only",
                })
    return events


def parse_simple_yaml(path):
    values = {}
    if not path:
        return values
    try:
        src = open(path, "r", encoding="utf-8", errors="replace")
    except OSError as error:
        print("warning: cannot read solver report {}: {}".format(path, error), file=sys.stderr)
        return values
    with src:
        for raw in src:
            match = re.match(r"^([A-Za-z_][A-Za-z0-9_]*):\s*(.*?)\s*$", raw)
            if not match:
                continue
            key, value = match.groups()
            if value.lower() in ("true", "false"):
                values[key] = value.lower() == "true"
            elif value.startswith('"') and value.endswith('"'):
                values[key] = value[1:-1].replace('\\"', '"').replace('\\\\', '\\')
            else:
                try:
                    values[key] = float(value) if any(c in value for c in ".eE") else int(value)
                except ValueError:
                    values[key] = value
    return values


def load_jsonl(path):
    events = []
    if not path:
        return events
    try:
        src = open(path, "r", encoding="utf-8", errors="replace")
    except OSError as error:
        print("warning: cannot read {}: {}".format(path, error), file=sys.stderr)
        return events
    with src:
        for number, raw in enumerate(src, 1):
            try:
                events.append(json.loads(raw))
            except json.JSONDecodeError as error:
                print("warning: {}:{}: {}".format(path, number, error), file=sys.stderr)
    return events


def ordering_time(event):
    value = event.get("monotonic")
    if isinstance(value, (int, float)):
        return float(value), "monotonic wall"
    value = event.get("wall_epoch")
    if isinstance(value, (int, float)):
        return float(value), "wall/ROS epoch"
    return None, None


def occurrence_time(event):
    """Best event occurrence wall time when /use_sim_time is false."""
    source = event.get("source", "")
    if source == "rosout_agg" and isinstance(event.get("ros_time"), (int, float)):
        return float(event["ros_time"]), "ROS occurrence time"
    if source == "recovery_snapshot_topic" and isinstance(
            event.get("header_stamp"), (int, float)):
        return float(event["header_stamp"]), "snapshot header wall time"
    if source == "nlp_recovery_trajectory_topic" and isinstance(
            event.get("ros_receive"), (int, float)):
        return float(event["ros_receive"]), "topic receive wall time"
    return None, None


def display_time(event):
    if event is None:
        return UNKNOWN
    occurrence, _ = occurrence_time(event)
    if occurrence is not None:
        return datetime.fromtimestamp(occurrence, timezone.utc).isoformat()
    if isinstance(event.get("utc"), str):
        return event["utc"]
    value = event.get("wall_epoch")
    if isinstance(value, (int, float)):
        return datetime.fromtimestamp(value, timezone.utc).isoformat()
    if isinstance(event.get("sim_time"), (int, float)):
        return "sim_t={:.3f}s".format(event["sim_time"])
    return UNKNOWN


def first_event(events, name):
    matches = [item for item in events if item.get("event") == name]
    if not matches:
        return None
    return sorted(matches, key=lambda item: ordering_time(item)[0]
                  if ordering_time(item)[0] is not None else float("inf"))[0]


def last_event(events, name):
    matches = [item for item in events if item.get("event") == name]
    if not matches:
        return None
    return sorted(matches, key=lambda item: ordering_time(item)[0]
                  if ordering_time(item)[0] is not None else float("-inf"))[-1]


def last_before(events, name, boundary):
    boundary_value = ordering_time(boundary)[0] if boundary else None
    matches = []
    for item in events:
        value = ordering_time(item)[0]
        if item.get("event") == name and value is not None and (
                boundary_value is None or value <= boundary_value):
            matches.append(item)
    return max(matches, key=lambda item: ordering_time(item)[0]) if matches else None


def interval(start, end):
    if start is None or end is None:
        return None
    start_value, start_clock = occurrence_time(start)
    end_value, end_clock = occurrence_time(end)
    if start_value is not None and end_value is not None:
        return end_value - start_value
    start_value, start_clock = ordering_time(start)
    end_value, end_clock = ordering_time(end)
    if start_value is None or end_value is None or start_clock != end_clock:
        return None
    return end_value - start_value


def fmt_duration(value):
    return UNKNOWN if value is None else "{:.6f} s".format(value)


def evidence(event):
    if event is None:
        return UNKNOWN
    return "{} ({})".format(event.get("source", "unknown"),
                            event.get("accuracy", "unspecified"))


def select_recovery(events, requested):
    ids = sorted({int(item["recovery_id"]) for item in events
                  if isinstance(item.get("recovery_id"), int) and item["recovery_id"] > 0})
    selected = requested if requested is not None else (ids[-1] if ids else None)
    if selected is None:
        return events, None
    explicit = [item for item in events if item.get("recovery_id") == selected]
    anchors = [ordering_time(item)[0] for item in explicit
               if ordering_time(item)[0] is not None]
    if not anchors:
        return explicit, selected
    lo, hi = min(anchors) - 120.0, max(anchors) + 120.0
    linked = []
    for item in events:
        rid = item.get("recovery_id")
        stamp = ordering_time(item)[0]
        if item.get("event") == "ipopt_reported_seconds":
            linked.append(item)
        elif rid == selected or (rid is None and stamp is not None and lo <= stamp <= hi):
            linked.append(item)
    return linked, selected


def build_report(events, solver, selected, inputs):
    confirmed = first_event(events, "deadlock_confirmed")
    candidate = last_before(events, "deadlock_candidate", confirmed)
    prep = first_event(events, "nlp_prep")
    freeze = first_event(events, "global_freeze")
    snapshot = first_event(events, "snapshot_received")
    guess_summary = first_event(events, "initial_guess_summary")
    guess_saved = first_event(events, "initial_guess_saved")
    solve_done = first_event(events, "solver_complete")
    result_saved = first_event(events, "result_saved")
    submitted_topic = first_event(events, "trajectory_topic_received")
    submitted_log = first_event(events, "trajectory_submitted")
    received = first_event(events, "trajectory_received")
    first_motion = first_event(events, "first_motion")
    ipopt_console = first_event(events, "ipopt_reported_seconds")

    rows = []

    def add(name, start, end, duration=None, note=""):
        measured = interval(start, end) if duration is None else duration
        source = "{} → {}".format(evidence(start), evidence(end))
        if note:
            source += "; " + note
        rows.append((name, display_time(start), display_time(end), fmt_duration(measured), source))

    add("死锁候选到确认", candidate, confirmed,
        note="候选仅有仿真时间时，Wall Time 必须保持 UNKNOWN")
    add("确认到 NLP_PREP", confirmed, prep)
    add("死锁候选到 NLP_PREP", candidate, prep)
    add("NLP_PREP 到全局冻结", prep, freeze)
    add("NLP_PREP 到 Snapshot 收到", prep, snapshot)
    add("输入/AABB + 初始解生成（合并可观测）", snapshot, guess_summary,
        note="现有外部边界不能拆分两者")
    add("输入与 AABB 构造", None, None,
        note="无独立外部时间边界")
    add("初始解生成", None, None,
        note="无独立外部时间边界")
    add("初始解文件保存", guess_summary, guess_saved)
    add("CasADi 建模 + nlpsol 初始化 + 求解 + 提取 + 验证（合并）",
        guess_saved, solve_done, note="只能测量整体包络")
    add("CasADi 建模", None, None, note="发生在 NlpSolver::solve() 内部计时起点之前")
    add("nlpsol 初始化", None, None, note="无外部边界")
    ipopt = solver.get("wall_time")
    add("CasADi solver(arguments) 数值调用", None, None,
        float(ipopt) if isinstance(ipopt, (int, float)) else None,
        "solver_report.yaml:t_wall_total；不等于 NlpSolver::solve() 总耗时")
    add("IPOPT 内部报告", None, None,
        float(ipopt_console["duration"]) if ipopt_console else None,
        "IPOPT console: Total seconds in IPOPT；无阶段时间戳")
    add("解向量提取", None, None, note="无外部边界")
    add("独立验证", None, None, note="在 solver log 前同步执行，无独立边界")
    add("结果保存", solve_done, result_saved)
    submit_boundary = submitted_topic or submitted_log
    add("结果保存后到轨迹提交", result_saved, submit_boundary)
    add("轨迹提交与规划器接收", submitted_topic or submitted_log, received,
        note="Topic 接收记录优先于发布后的日志行")
    add("接收到首次车辆运动", received or submitted_topic, first_motion)
    add("合计（候选到首次运动）", candidate, first_motion)
    add("合计（NLP_PREP 到首次运动）", prep, first_motion)

    measurable = []
    for name, _, _, value, _ in rows:
        if value != UNKNOWN:
            try:
                measurable.append((float(value.split()[0]), name))
            except ValueError:
                pass
    measurable.sort(reverse=True)

    lines = [
        "# 多叉车死锁至 NLP 恢复运动耗时报告",
        "",
        "- recovery_id: `{}`".format(selected if selected is not None else "UNKNOWN"),
        "- 生成时间（UTC）: `{}`".format(datetime.now(timezone.utc).isoformat()),
        "- 输入证据: {}".format(", ".join(inputs) if inputs else "none"),
        "- 时钟规则: collector 的 `monotonic` 用于墙钟差；rosout 时间仅在 `/use_sim_time=false` 时可视为墙钟兼容；诊断文件中的 `sim_t` 绝不当作墙钟。",
        "",
        "| 阶段 | 开始时间 | 结束时间 | Wall Time | 证据来源 |",
        "| --- | --- | --- | ---: | --- |",
    ]
    for row in rows:
        lines.append("| {} | {} | {} | {} | {} |".format(
            *(str(item).replace("|", "\\|") for item in row)))

    lines.extend(["", "## 求解报告", ""])
    if solver:
        for key in ("solver_success", "validation_passed", "return_status",
                    "iterations", "wall_time", "cpu_time", "node_count",
                    "total_time", "average_dt", "max_constraint_violation"):
            lines.append("- {}: `{}`".format(key, solver.get(key, UNKNOWN)))
    else:
        lines.append("- 未提供或未找到 `solver_report.yaml`。")

    lines.extend(["", "## 按实测耗时排序", ""])
    if measurable:
        for value, name in measurable:
            lines.append("- {:.6f} s — {}".format(value, name))
    else:
        lines.append("- 没有足够的同一时钟证据可排序。")

    lines.extend([
        "", "## 可观测性结论", "",
        "- `solver_report.wall_time` 来自 `solver.stats().t_wall_total`，覆盖 `solver(arguments)` 的 CasADi 数值调用；符号模型和 `nlpsol()` 创建在它之前，解提取和独立验证在它之后执行。它不等同于 IPOPT 控制台的 `Total seconds in IPOPT`。",
        "- 恢复节点使用 `ros::spin()` 单线程回调；输入/AABB、debug input 写盘、初始解、case 写盘、求解、验证和结果写盘都在 Snapshot 回调中串行发生，保存完成后才发布轨迹。",
        "- 规划器接收轨迹时立即设置 `nlp_execution_active_`；车辆状态在下一次 timer 回调的 `executeNlpRecovery()` 中更新。`simulation_speed` 决定每个墙钟 timer 回调推进多少个 0.1 s 仿真步，不是 NLP 轨迹的墙钟时长。",
        "- 任何显示为 UNKNOWN 的阶段都不能用总等待减去已知耗时来填补。",
    ])
    return "\n".join(lines) + "\n"


def analyze(args):
    events = load_jsonl(args.events)
    inputs = []
    if args.events:
        inputs.append(args.events)
    for path in args.ros_log:
        events.extend(parse_ros_log(path))
        inputs.append(path)
    for path in args.diagnostic_log:
        events.extend(parse_diagnostic_log(path))
        inputs.append(path)
    solver = parse_simple_yaml(args.solver_report)
    if args.solver_report:
        inputs.append(args.solver_report)
    selected_events, selected = select_recovery(events, args.recovery_id)
    report = build_report(selected_events, solver, selected, inputs)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as out:
        out.write(report)
    print(args.output)
    return 0


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    collect_parser = sub.add_parser("collect", help="subscribe read-only and write JSONL")
    collect_parser.add_argument("--output", required=True)
    collect_parser.add_argument("--timeout", type=float, default=900.0)
    collect_parser.add_argument("--stop-after-motion", action="store_true")
    collect_parser.add_argument("--diagnostics-root")
    collect_parser.add_argument("--position-epsilon", type=float, default=1.0e-5)
    collect_parser.add_argument("--speed-epsilon", type=float, default=1.0e-5)
    collect_parser.set_defaults(func=collect)

    analyze_parser = sub.add_parser("analyze", help="create a Markdown stage report")
    analyze_parser.add_argument("--events")
    analyze_parser.add_argument("--ros-log", action="append", default=[])
    analyze_parser.add_argument("--diagnostic-log", action="append", default=[])
    analyze_parser.add_argument("--solver-report")
    analyze_parser.add_argument("--recovery-id", type=int)
    analyze_parser.add_argument("--output", required=True)
    analyze_parser.set_defaults(func=analyze)
    return parser


def main():
    args = make_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
