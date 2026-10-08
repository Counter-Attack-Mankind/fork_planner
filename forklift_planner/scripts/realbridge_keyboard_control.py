#!/usr/bin/env python3
"""Terminal keyboard control for the existing realbridge start/estop topics."""

import atexit
import select
import sys
import termios
import time
import tty

import rospy
from std_msgs.msg import Bool


class RealbridgeKeyboardControl:
    def __init__(self):
        self._start_pub = rospy.Publisher("/rb_start", Bool, queue_size=1, latch=True)
        self._estop_pub = rospy.Publisher("/estop", Bool, queue_size=1, latch=True)
        self._stop_published = False

        rospy.on_shutdown(self.publish_stop)
        atexit.register(self.publish_stop)

        # /rb_start=false is ignored by the current planner.  /estop=true is
        # therefore the signal that actually forces every real vehicle to stop.
        self.publish_stop()

    def publish_stop(self):
        if self._stop_published:
            return
        self._stop_published = True
        try:
            self._estop_pub.publish(Bool(data=True))
            self._start_pub.publish(Bool(data=False))
        except Exception as exc:  # Best effort during interpreter/ROS shutdown.
            try:
                rospy.logerr("Failed to publish final realbridge stop: %s", exc)
            except Exception:
                pass

    def start(self):
        self._stop_published = False
        # /rb_start remains the existing one-way start permission and is never
        # treated as the stopping mechanism.
        self._estop_pub.publish(Bool(data=False))
        self._start_pub.publish(Bool(data=True))

    def stop(self):
        self._stop_published = False
        self.publish_stop()


class TerminalInput:
    def __init__(self):
        self._stream = None
        self._fd = None
        self._saved_attributes = None

    def __enter__(self):
        if sys.stdin.isatty():
            self._stream = sys.stdin
        else:
            # roslaunch may redirect a node's stdin.  Open the controlling
            # terminal explicitly so the standalone launch remains interactive.
            self._stream = open("/dev/tty", "r", encoding="utf-8")

        self._fd = self._stream.fileno()
        self._saved_attributes = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self._saved_attributes is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved_attributes)
        if self._stream is not None and self._stream is not sys.stdin:
            self._stream.close()

    def read_key(self, timeout):
        readable, _, _ = select.select([self._stream], [], [], timeout)
        return self._stream.read(1) if readable else None


def show_status(running):
    if running:
        message = "[RUNNING] Press 0 to EMERGENCY STOP"
    else:
        message = "[STOPPED] Press 1 to START"
    sys.stdout.write("\r\033[2K" + message)
    sys.stdout.flush()


def main():
    rospy.init_node("realbridge_keyboard_control")
    control = RealbridgeKeyboardControl()
    running = False

    try:
        with TerminalInput() as terminal:
            show_status(running)
            while not rospy.is_shutdown():
                key = terminal.read_key(0.1)
                if key == "1":
                    control.start()
                    running = True
                    show_status(running)
                elif key == "0":
                    control.stop()
                    running = False
                    show_status(running)
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    except Exception as exc:
        rospy.logerr("Realbridge keyboard control failed: %s", exc)
    finally:
        control.stop()
        # Give connected subscribers a brief opportunity to receive the final
        # stop before this publisher process exits.
        time.sleep(0.1)
        sys.stdout.write("\n[STOPPED] Keyboard control exited\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
