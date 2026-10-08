#!/usr/bin/env python3
"""Replay one bag or every bag segment in a snapshot directory."""

import glob
import os
import signal
import subprocess
import time

import rospy


class SnapshotReplay:
    def __init__(self):
        snapshot = os.path.abspath(os.path.expanduser(rospy.get_param("~snapshot")))
        if os.path.isdir(snapshot):
            bags = sorted(glob.glob(os.path.join(snapshot, "*.bag")))
        elif glob.has_magic(snapshot):
            bags = sorted(glob.glob(snapshot))
        else:
            bags = [snapshot] if os.path.isfile(snapshot) else []
        if not bags:
            raise RuntimeError("no .bag files found for snapshot: %s" % snapshot)

        rate = max(0.01, float(rospy.get_param("~rate", 0.25)))
        paused = bool(rospy.get_param("~paused", False))
        command = ["rosbag", "play", "--clock", "--rate", str(rate)]
        if paused:
            command.append("--pause")
        command.extend(bags)
        rospy.loginfo("[RVIZ-REPLAY] playing %d segment(s) at %.3fx", len(bags), rate)
        self.process = subprocess.Popen(command, preexec_fn=os.setsid)
        rospy.on_shutdown(self.shutdown)

    def wait(self):
        while not rospy.is_shutdown() and self.process.poll() is None:
            # /use_sim_time is true during replay; wall time must be used here
            # so the wrapper can notice EOF after the final /clock message.
            time.sleep(0.1)
        if self.process.poll() not in (None, 0):
            raise RuntimeError("rosbag play exited with code %d" % self.process.returncode)

    def shutdown(self):
        if self.process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception:
                pass


if __name__ == "__main__":
    rospy.init_node("rviz_snapshot_replay")
    replay = SnapshotReplay()
    replay.wait()
