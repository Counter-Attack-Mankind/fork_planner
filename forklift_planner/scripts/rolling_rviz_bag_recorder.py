#!/usr/bin/env python3
"""Keep short rosbag segments and archive the rolling window on a trigger."""

import glob
import os
import queue
import re
import shutil
import signal
import subprocess
import threading
import time

import rospy
from std_msgs.msg import String


class RollingRvizBagRecorder:
    def __init__(self):
        self.output_dir = os.path.abspath(
            os.path.expanduser(rospy.get_param("~output_dir", "~/forklift_debug_bags")))
        self.segment_duration = max(1.0, float(rospy.get_param("~segment_duration", 10.0)))
        # rosbag keeps max_splits finalized files plus the active segment. With
        # 10-second segments, 11 finalized + the active file is about 120 s.
        self.max_splits = max(2, int(rospy.get_param("~max_splits", 11)))
        self.topics = list(rospy.get_param(
            "~topics", ["/forklift_map/markers", "/forklift_planner/markers"]))
        self.trigger_topic = rospy.get_param(
            "~trigger_topic", "/forklift_planner/debug_snapshot_trigger")
        self.finalize_timeout = max(2.0, float(rospy.get_param("~finalize_timeout", 15.0)))

        self.spool_dir = os.path.join(self.output_dir, "rolling")
        self.archive_dir = os.path.join(self.output_dir, "snapshots")
        os.makedirs(self.spool_dir, exist_ok=True)
        os.makedirs(self.archive_dir, exist_ok=True)
        self.session = "rolling_%d_%d" % (int(time.time()), os.getpid())
        self.prefix = os.path.join(self.spool_dir, self.session)
        self.process = None
        self.lock = threading.Lock()
        self.events = queue.Queue()
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()
        self._start_recorder()
        self.subscription = rospy.Subscriber(
            self.trigger_topic, String, self._trigger, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def _command(self):
        return [
            "rosbag", "record", "--lz4", "--repeat-latched", "--split",
            "--duration=%.3f" % self.segment_duration,
            "--max-splits=%d" % self.max_splits,
            "-o", self.prefix,
        ] + self.topics

    def _start_recorder(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return
            try:
                self.process = subprocess.Popen(
                    self._command(), preexec_fn=os.setsid,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                rospy.loginfo("[RVIZ-BAG] rolling recorder pid=%d topics=%s",
                              self.process.pid, ",".join(self.topics))
            except Exception as error:  # Recorder failure must not affect coordination.
                self.process = None
                rospy.logerr("[RVIZ-BAG] failed to start rosbag: %s", error)

    def _stop_recorder(self):
        with self.lock:
            process = self.process
            self.process = None
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=self.finalize_timeout)
        except subprocess.TimeoutExpired:
            rospy.logerr("[RVIZ-BAG] rosbag finalize timeout; terminating recorder")
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    pass
        except Exception as error:
            rospy.logerr("[RVIZ-BAG] failed to stop recorder: %s", error)

    @staticmethod
    def _fields(text):
        return dict(re.findall(r"([A-Za-z_]+)=([^\s]+)", text))

    @staticmethod
    def _safe(value):
        return re.sub(r"[^A-Za-z0-9_.-]+", "-", value)

    def _trigger(self, message):
        # Keep the ROS callback non-blocking. Finalization and filesystem work
        # happen in the worker thread, outside the coordinator process.
        self.events.put(message.data)

    def _worker(self):
        while not rospy.is_shutdown():
            try:
                event_text = self.events.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._archive(event_text)
            except Exception as error:
                rospy.logerr("[RVIZ-BAG] snapshot archive failed: %s", error)
            finally:
                self.session = "rolling_%d_%d" % (int(time.time()), os.getpid())
                self.prefix = os.path.join(self.spool_dir, self.session)
                self._start_recorder()
                self.events.task_done()

    def _archive(self, event_text):
        old_prefix = self.prefix
        self._stop_recorder()
        fields = self._fields(event_text)
        stem = "{event}_seed{seed}_v{count}_sim{sim}_tick{tick}".format(
            event=self._safe(fields.get("event", "EVENT")),
            seed=self._safe(fields.get("seed", "unknown")),
            count=self._safe(fields.get("vehicle_count", "unknown")),
            sim=self._safe(fields.get("sim_t", "unknown")),
            tick=self._safe(fields.get("tick", "unknown")))
        destination = os.path.join(self.archive_dir, stem)
        suffix = 1
        while os.path.exists(destination):
            destination = os.path.join(self.archive_dir, "%s_%02d" % (stem, suffix))
            suffix += 1
        os.makedirs(destination)

        bags = sorted(glob.glob(old_prefix + "*.bag"))
        for index, source in enumerate(bags):
            target = os.path.join(destination, "%s_part%03d.bag" % (stem, index))
            shutil.move(source, target)
        active = glob.glob(old_prefix + "*.bag.active")
        if active:
            rospy.logwarn("[RVIZ-BAG] ignored %d unfinalized active file(s)", len(active))
        with open(os.path.join(destination, "trigger.txt"), "w", encoding="utf-8") as output:
            output.write(event_text + "\n")
            output.write("topics=" + ",".join(self.topics) + "\n")
            output.write("segment_duration=%.3f\n" % self.segment_duration)
            output.write("max_splits=%d\n" % self.max_splits)
        rospy.logwarn("[RVIZ-BAG] archived %d segment(s): %s", len(bags), destination)

    def shutdown(self):
        self._stop_recorder()


if __name__ == "__main__":
    rospy.init_node("rolling_rviz_bag_recorder")
    RollingRvizBagRecorder()
    rospy.spin()
