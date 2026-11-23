#!/usr/bin/env python3
"""Live health monitor for rosbag recording.

Runs `ros2 bag record` (everything after `--`) and, alongside it, subscribes to
the same topics to flag low rates, arrival gaps (dropped messages) and stale
topics. Recorder log lines reporting lost/dropped messages are flagged too.

  record_monitor.py -- ros2 bag record --topics /a /b ... -d 30 ...
  record_monitor.py                      # monitor only, no recording
  record_monitor.py --check-bag <bag_dir>  # offline check of a finished bag

After a recording stops, prompts for parking labels and appends one row to
parking_labels.csv next to the bag (one CSV per day folder).
"""

import argparse
import csv
import os
import re
import signal
import subprocess
import sys
import termios
import threading
import time
from collections import deque
from datetime import datetime

# topic -> expected rate in Hz. None = no fixed rate (only warn when silent),
# "latched" = expect at least one message (e.g. /tf_static).
# Rates measured on the vehicle with all drivers running.
EXPECTED = {
    "/camera_fl/image_color/compressed": 15.0,
    "/camera_fr/image_color/compressed": 15.0,
    "/camera_rear_1/image_color/compressed": 15.0,
    "/camera_rear_2/image_color/compressed": 15.0,
    "/camera_rear_3/image_color/compressed": 15.0,
    "/camera_rear_4/image_color/compressed": 15.0,
    "/camera_rear_5/image_color/compressed": 15.0,
    "/camera_fl/camera_info": 15.0,
    "/camera_fr/camera_info": 15.0,
    "/camera_rear_1/camera_info": 15.0,
    "/camera_rear_2/camera_info": 15.0,
    "/camera_rear_3/camera_info": 15.0,
    "/camera_rear_4/camera_info": 15.0,
    "/camera_rear_5/camera_info": 15.0,
    "/lidar_tc/velodyne_packets": 10.0,
    "/lidar_tl/velodyne_packets": 10.0,
    "/lidar_tr/velodyne_packets": 10.0,
    "/tf": 20.0,
    "/tf_static": "latched",
    "/novatel/oem7/odom": 100.0,
    "/novatel/oem7/odom_grid": None,
    "/novatel/oem7/imu/data_raw": 200.0,
    "/r/raptor_dbw_interface/surround_report": None,
    "/delphi_esr_interface/radar/status": None,
    "/delphi_esr_interface/radar/tracks": None,
}

RECORDER_ALERT = re.compile(r"lost|drop|exceed|error|fail", re.IGNORECASE)
# rmw_zenoh logs this as an ERROR when a message arrives while the recorder is
# tearing its subscriptions down on Ctrl-C; it is harmless shutdown noise.
RECORDER_IGNORE = re.compile(r"SubscriberCallback triggered over")

RED, YELLOW, GREEN, DIM, RESET = "\033[31m", "\033[33m", "\033[32m", "\033[2m", "\033[0m"

LABEL_CSV = "parking_labels.csv"
DIRECTIONS = ("forward", "reverse")
STYLES = ("regular", "angled", "indoor", "parallel")


class TopicStats:
    def __init__(self, topic, expected, window_s):
        self.topic = topic
        self.expected = expected
        self.window_s = window_s
        self.arrivals = deque()
        self.count = 0
        self.first = None
        self.last = None
        self.gaps = 0
        self.est_missed = 0
        self.min_rate = None
        self.low_s = 0.0
        self.state = "WAIT"

    @property
    def rated(self):
        return isinstance(self.expected, float)

    def on_msg(self, now):
        gap = None
        if self.last is not None and self.rated:
            period = 1.0 / self.expected
            dt = now - self.last
            # Arrival jitter is real on a loaded machine; only call it a gap when
            # the hole is both several periods and >=100 ms long.
            if dt > max(3.0 * period, 0.1):
                self.gaps += 1
                self.est_missed += max(0, round(dt / period) - 1)
                gap = dt
        if self.first is None:
            self.first = now
        self.last = now
        self.count += 1
        self.arrivals.append(now)
        return gap

    def rate(self, now):
        while self.arrivals and self.arrivals[0] < now - self.window_s:
            self.arrivals.popleft()
        if self.first is None:
            return 0.0
        span = min(self.window_s, now - self.first)
        return len(self.arrivals) / span if span > 0 else 0.0


def check_bag(bag_dir, min_ratio):
    import yaml

    with open(os.path.join(bag_dir, "metadata.yaml")) as f:
        info = yaml.safe_load(f)["rosbag2_bagfile_information"]
    duration = info["duration"]["nanoseconds"] / 1e9
    print(f"Bag: {bag_dir}  duration {duration:.1f} s  files {len(info.get('relative_file_paths', []))}\n")
    print(f"{'topic':48} {'msgs':>8} {'rate':>8} {'expect':>7}  status")
    bad = 0
    recorded = {}
    for t in info["topics_with_message_count"]:
        recorded[t["topic_metadata"]["name"]] = t["message_count"]
    for topic in sorted(set(EXPECTED) | set(recorded)):
        n = recorded.get(topic)
        exp = EXPECTED.get(topic)
        if n is None:
            # Topics without a fixed rate may legitimately be absent (sensor off).
            if isinstance(exp, float):
                status, bad = f"{RED}NOT RECORDED{RESET}", bad + 1
            else:
                status = f"{YELLOW}NOT RECORDED{RESET}"
            print(f"{topic:48} {'-':>8} {'-':>8} {str(exp):>7}  {status}")
            continue
        rate = n / duration if duration > 0 else 0.0
        if isinstance(exp, float):
            ok = rate >= exp * min_ratio
            missing = max(0, round(exp * duration) - n)
            status = f"{GREEN}OK{RESET}" if ok else f"{RED}LOW (~{missing} missing){RESET}"
            bad += not ok
        else:
            ok = n > 0
            status = f"{GREEN}OK{RESET}" if ok else f"{YELLOW}EMPTY{RESET}"
        print(f"{topic:48} {n:>8} {rate:>8.2f} {str(exp if exp is not None else '-'):>7}  {status}")
    return 1 if bad else 0


def bag_output(record_cmd):
    """Return (parent_dir, explicit_name_or_None) for where the recorder writes."""
    output = None
    for i, tok in enumerate(record_cmd):
        if tok in ("-o", "--output") and i + 1 < len(record_cmd):
            output = record_cmd[i + 1]
        elif tok.startswith("--output="):
            output = tok.split("=", 1)[1]
    if output is None:
        return os.getcwd(), None
    path = os.path.abspath(output)
    return os.path.dirname(path), os.path.basename(path)


def subdirs(path):
    try:
        return {e.name for e in os.scandir(path) if e.is_dir()}
    except OSError:
        return set()


def find_new_bag(parent, name, before):
    new = subdirs(parent) - before
    if name:
        return name if name in new else None
    bags = sorted(d for d in new if d.startswith("rosbag2_"))
    return bags[-1] if bags else None


def ask_choice(prompt, valid):
    while True:
        answer = input(prompt).strip()
        if not answer:
            return None
        if answer in valid:
            return int(answer)
        print(f"  please enter one of: {', '.join(valid)}, or leave blank")


def label_bag(parent, bag_name, tty_attrs):
    """Prompt for this bag's parking labels and append them to the folder's CSV."""
    if bag_name is None:
        print(f"{YELLOW}no new bag folder found, nothing to label{RESET}")
        return
    if tty_attrs is None:
        print(f"{YELLOW}no terminal to prompt on, {bag_name} was not labeled{RESET}")
        return

    # The recorder shares this terminal and can leave echo off if it was killed.
    # TCSAFLUSH also drops anything typed during the recording.
    termios.tcsetattr(sys.stdin, termios.TCSAFLUSH, tty_attrs)
    # The recording handlers swallow Ctrl-C, which would trap the user in a prompt.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)

    csv_path = os.path.join(parent, LABEL_CSV)
    print(f"\n{GREEN}Label {bag_name}{RESET}  (Ctrl-C to skip)")
    try:
        entry_exit = ask_choice("Entry or exit?  0=entry 1=exit (Enter for none): ", ("0", "1"))
        direction = ask_choice("Direction?  0=forward 1=reverse (Enter for none): ", ("0", "1"))
        style = ask_choice("Style?  0=regular 1=angled 2=indoor 3=parallel (Enter for none): ",
                           ("0", "1", "2", "3"))
        notes = input("Notes (Enter for none): ").strip()
    except (KeyboardInterrupt, EOFError):
        print(f"\n{YELLOW}skipped, {bag_name} was not labeled{RESET}")
        return

    is_new = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        if is_new:
            writer.writerow(["rosbag_name", "entry", "exit", "direction", "style", "notes"])
        writer.writerow([bag_name, "x" if entry_exit == 0 else "", "x" if entry_exit == 1 else "",
                         "" if direction is None else DIRECTIONS[direction],
                         "" if style is None else STYLES[style], notes])
    print(f"saved to {csv_path}")


def main():
    if "--" in sys.argv:
        i = sys.argv.index("--")
        own_args, record_cmd = sys.argv[1:i], sys.argv[i + 1:]
    else:
        own_args, record_cmd = sys.argv[1:], []

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--min-ratio", type=float, default=0.9, help="flag LOW below this fraction of expected rate")
    ap.add_argument("--window", type=float, default=5.0, help="rate window in seconds")
    ap.add_argument("--stale", type=float, default=2.0, help="seconds without messages before STALE")
    ap.add_argument("--period", type=float, default=2.0, help="seconds between status refreshes")
    ap.add_argument("--warmup", type=float, default=5.0, help="seconds before alerts start")
    ap.add_argument("--log", default=None, help="CSV event log (default: record_monitor_<time>.csv)")
    ap.add_argument("--check-bag", metavar="BAG_DIR", help="check a finished bag's counts and exit")
    args = ap.parse_args(own_args)

    if args.check_bag:
        sys.exit(check_bag(args.check_bag, args.min_ratio))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.utilities import get_message

    topics = EXPECTED
    if record_cmd and "--topics" in record_cmd:
        # Monitor exactly what is being recorded.
        listed = []
        for tok in record_cmd[record_cmd.index("--topics") + 1:]:
            if tok.startswith("-"):
                break
            listed.append(tok)
        topics = {t: EXPECTED.get(t) for t in listed}

    log_path = args.log or datetime.now().strftime("record_monitor_%Y%m%d_%H%M%S.csv")
    log_file = open(log_path, "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["wall_time", "elapsed_s", "topic", "event", "detail"])
    lock = threading.Lock()
    events = deque(maxlen=12)
    t0 = time.monotonic()

    def event(topic, kind, detail, color=YELLOW):
        el = time.monotonic() - t0
        with lock:
            log.writerow([datetime.now().isoformat(timespec="milliseconds"), f"{el:.2f}", topic, kind, detail])
            log_file.flush()
            events.append(f"{color}{datetime.now():%H:%M:%S} {kind:9}{RESET} {topic} {detail}")

    rec = None
    rec_done = threading.Event()
    if record_cmd:
        bag_parent, bag_name = bag_output(record_cmd)
        dirs_before = subdirs(bag_parent)
        tty_attrs = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None

        # Own session so a terminal Ctrl-C reaches only us; we forward exactly one
        # SIGINT so the recorder shuts down cleanly and finishes writing.
        rec = subprocess.Popen(record_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, bufsize=1, start_new_session=True)

        def pump():
            for line in rec.stdout:
                line = line.rstrip()
                if RECORDER_ALERT.search(line) and not RECORDER_IGNORE.search(line):
                    event("recorder", "RECORDER", line, RED)
            rec_done.set()

        threading.Thread(target=pump, daemon=True).start()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("record_monitor")
    stats = {t: TopicStats(t, e, args.window) for t, e in topics.items()}
    subscribed = set()
    split_count = [0]
    stop_time = [None]

    def make_cb(st):
        def cb(_):
            gap = st.on_msg(time.monotonic())
            if gap is not None and time.monotonic() - t0 > args.warmup:
                event(st.topic, "GAP", f"{gap * 1000:.0f} ms without messages (~{round(gap * st.expected) - 1} missed)")
        return cb

    def try_subscribe():
        types = dict(node.get_topic_names_and_types())
        for t, st in stats.items():
            if t in subscribed or t not in types:
                continue
            msg_cls = get_message(types[t][0])
            if st.expected == "latched":
                qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            else:
                # Best effort: never back-pressures publishers or the recorder.
                qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(msg_cls, t, make_cb(st), qos, raw=True)
            subscribed.add(t)

    try:
        from rosbag2_interfaces.msg import WriteSplitEvent

        def on_split(msg):
            split_count[0] += 1
            event("recorder", "SPLIT", os.path.basename(msg.closed_file), DIM)

        node.create_subscription(WriteSplitEvent, "/events/write_split", on_split, 10)
    except ImportError:
        pass

    def report():
        try_subscribe()
        now = time.monotonic()
        warm = now - t0 > args.warmup
        rows = []
        n_bad = 0
        for t, st in stats.items():
            rate = st.rate(now)
            age = now - st.last if st.last else None
            if st.expected == "latched":
                state = "OK" if st.count else "WAIT"
            elif st.last is None:
                state = "MISSING" if warm else "WAIT"
            elif age > args.stale:
                state = "STALE"
            elif st.rated and warm and rate < st.expected * args.min_ratio:
                state = "LOW"
            else:
                state = "OK"
            if st.rated and warm and st.last is not None:
                st.min_rate = rate if st.min_rate is None else min(st.min_rate, rate)
                if state in ("LOW", "STALE"):
                    st.low_s += args.period
            if warm and state != st.state and not (st.expected is None and state == "MISSING"):
                if state in ("LOW", "STALE", "MISSING"):
                    event(t, state, f"rate {rate:.1f} Hz" + (f" (expected {st.expected:g})" if st.rated else ""), RED)
                elif st.state in ("LOW", "STALE", "MISSING"):
                    event(t, "RECOVERED", f"rate {rate:.1f} Hz", GREEN)
            st.state = state
            bad = state in ("LOW", "STALE") or (state == "MISSING" and st.expected is not None)
            n_bad += bad
            color = RED if bad else (DIM if state in ("WAIT", "MISSING") else GREEN)
            exp = f"{st.expected:g}" if st.rated else ("-" if st.expected is None else "latch")
            rows.append(f"{t:46} {exp:>6} {rate:7.1f} {st.gaps:5d} {st.est_missed:7d}  {color}{state}{RESET}")

        el = int(now - t0)
        if rec is None:
            rec_state = "not started"
        elif rec.poll() is not None:
            rec_state = "stopped"
        elif stop_time[0] is not None:
            rec_state = (f"{YELLOW}STOPPING {int(now - stop_time[0])}s - finishing/compressing last file, "
                         f"Ctrl-C again to force{RESET}")
        else:
            rec_state = "running (Ctrl-C to stop)"
        out = ["\033[H\033[J" if sys.stdout.isatty() else "",
               f"record_monitor  {el // 60:02d}:{el % 60:02d}  recorder: {rec_state}  "
               f"splits: {split_count[0]}  issues: {RED if n_bad else GREEN}{n_bad}{RESET}  log: {log_path}",
               f"{'topic':46} {'expect':>6} {'rate':>7} {'gaps':>5} {'missed':>7}  state"]
        out += rows
        with lock:
            if events:
                out += ["", "recent events:"] + list(events)
        print("\n".join(out), flush=True)

    node.create_timer(args.period, report)

    # 1st Ctrl-C: SIGINT the recorder and keep monitoring until it has flushed,
    # compressed the last split and exited (its "lost messages" summary is
    # printed then). 2nd: SIGTERM. 3rd: SIGKILL - the last split may be lost.
    stop_signals = [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
    stop_count = [0]

    def on_sigint(*_):
        if stop_time[0] is None:
            stop_time[0] = time.monotonic()
        if rec is not None and rec.poll() is None and stop_count[0] < len(stop_signals):
            sig = stop_signals[stop_count[0]]
            try:
                os.killpg(rec.pid, sig)
            except ProcessLookupError:
                pass
            event("recorder", "STOP", f"sent {signal.Signals(sig).name}", YELLOW if stop_count[0] == 0 else RED)
        stop_count[0] += 1

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            if rec is None and stop_time[0] is not None:
                break
            if rec is not None and rec.poll() is not None and rec_done.wait(1.0):
                break
    finally:
        rc = rec.wait() if rec is not None else 0
        print("\nsummary:")
        print(f"{'topic':46} {'msgs':>8} {'min rate':>8} {'gaps':>5} {'missed':>7} {'low s':>6}")
        problems = 0
        for t, st in stats.items():
            mr = f"{st.min_rate:.1f}" if st.min_rate is not None else "-"
            flag = st.gaps or st.low_s or (st.count == 0 and st.expected is not None)
            problems += bool(flag)
            print(f"{(RED if flag else '')}{t:46} {st.count:>8} {mr:>8} {st.gaps:5d} {st.est_missed:7d} {st.low_s:6.0f}{RESET}")
        if rec is not None:
            print(f"recorder exit code: {rc}")
        print(f"event log: {log_path}")
        log_file.close()
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RuntimeError as e:  # rmw_zenoh can time out closing its session
            print(f"warning: rclpy shutdown: {e}")
        if rec is not None:
            label_bag(bag_parent, find_new_bag(bag_parent, bag_name, dirs_before), tty_attrs)
        sys.exit(1 if problems or rc else 0)


if __name__ == "__main__":
    main()
