#!/usr/bin/env python3
import csv
import json
import time
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple

import rospy
import yaml
from roslib.message import get_message_class


def now_sec() -> float:
    return rospy.Time.now().to_sec()

def header_stamp_to_sec(msg, header_field: str = "header.stamp") -> Optional[float]:
    try:
        obj = msg
        for part in header_field.split("."):
            obj = getattr(obj, part)
        return obj.to_sec()
    except Exception:
        return None

@dataclass
class SensorCfg:
    sid: str
    raw_topic: str
    converted_topic: str
    converted_type: str
    many_to_one: bool = False
    one_to_many: bool = False
    use_arrival_time: bool = False
    header_field: str = "header.stamp"
    required: bool = True

class ConversionValidator:
    def __init__(self):
        cfg_path = rospy.get_param("~sensors_config", "config/sensors.yaml")
        self.tolerance_ms = float(rospy.get_param("~tolerance_ms", 50.0))
        self.threshold = float(rospy.get_param("~threshold", 99.0))
        self.queue_size = int(rospy.get_param("~queue_size", 100))
        self.log_dir = rospy.get_param("~log_dir", "/tmp/teo2_005_logs")

        self.tolerance_s = self.tolerance_ms / 1000.0
        self.start_ts = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"{self.log_dir}/conversion_events_{self.start_ts}.csv"
        self.json_path = f"{self.log_dir}/summary_{self.start_ts}.json"

        rospy.loginfo(f"[validator] tolerance_ms={self.tolerance_ms} threshold={self.threshold}")
        rospy.loginfo(f"[validator] logs at {self.log_dir}")

        import os
        os.makedirs(self.log_dir, exist_ok=True)

        with open(cfg_path, "r") as f:
            cfg = yaml.safe_load(f)

        self.sensors: Dict[str, SensorCfg] = {}
        for s in cfg.get("sensors", []):
            self.sensors[s["id"]] = SensorCfg(
                sid=s["id"],
                raw_topic=s["raw_topic"],
                converted_topic=s["converted_topic"],
                converted_type=s["converted_type"],
                many_to_one=s.get("many_to_one", False),
                one_to_many=s.get("one_to_many", False),
                use_arrival_time=s.get("use_arrival_time", False),
                header_field=s.get("header_field", "header.stamp"),
                required=s.get("required", True),
            )

        self.raw_buf: Dict[str, Deque[float]] = defaultdict(lambda: deque(maxlen=10000))
        self.conv_buf: Dict[str, Deque[float]] = defaultdict(lambda: deque(maxlen=10000))

        self.attempts: Dict[str, int] = defaultdict(int)
        self.successes: Dict[str, int] = defaultdict(int)

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["event_time", "sensor_id", "raw_time", "converted_time", "delta_ms", "matched", "reason"])

        for sid, sc in self.sensors.items():
            rospy.Subscriber(sc.raw_topic, rospy.AnyMsg, self._raw_cb, callback_args=sid, queue_size=self.queue_size)
            msg_cls = get_message_class(sc.converted_type)
            if msg_cls is None:
                rospy.logwarn(f"[{sid}] Unknown type {sc.converted_type}; subscribing AnyMsg.")
                rospy.Subscriber(sc.converted_topic, rospy.AnyMsg, self._conv_cb, callback_args=sid, queue_size=self.queue_size)
            else:
                rospy.Subscriber(sc.converted_topic, msg_cls, self._conv_cb, callback_args=sid, queue_size=self.queue_size)

        rospy.on_shutdown(self.on_shutdown)
        rospy.Timer(rospy.Duration(1.0), self._periodic_match)

    def _raw_cb(self, msg, sid: str):
        t = now_sec() if self.sensors[sid].use_arrival_time else (header_stamp_to_sec(msg) or now_sec())
        self.raw_buf[sid].append(t)
        self.attempts[sid] += 1
        self._try_match(sid)

    def _conv_cb(self, msg, sid: str):
        sc = self.sensors[sid]
        t = now_sec() if sc.use_arrival_time else (header_stamp_to_sec(msg, sc.header_field) or now_sec())
        self.conv_buf[sid].append(t)
        self._try_match(sid)

    def _try_match(self, sid: str):
        rb = self.raw_buf[sid]
        cb = self.conv_buf[sid]
        matched_any = False

        def pop_nearest(a: Deque[float], b: Deque[float]):
            if not a or not b:
                return None
            ra = a[0]
            nearest_idx, nearest_dt = None, None
            for i, bt in enumerate(b):
                dt = abs(bt - ra)
                if nearest_dt is None or dt < nearest_dt:
                    nearest_dt, nearest_idx = dt, i
            if nearest_idx is None:
                return None
            bt = b[nearest_idx]
            if nearest_dt <= self.tolerance_s:
                a.popleft()
                b.rotate(-nearest_idx); b.popleft(); b.rotate(nearest_idx)
                return ra, bt, nearest_dt
            return None

        while rb and cb:
            res = pop_nearest(rb, cb)
            if res is None:
                break
            raw_t, conv_t, dt = res
            self.successes[sid] += 1
            self._log_event(sid, raw_t, conv_t, dt*1000.0, True, "matched")
            matched_any = True

        if not matched_any and len(rb) > 0 and len(cb) > 0:
            dt = abs(cb[0] - rb[0])
            if dt > self.tolerance_s:
                self._log_event(sid, rb[0], cb[0], dt*1000.0, False, "outside_tolerance_pending")

    def _periodic_match(self, _evt):
        for sid in list(self.sensors.keys()):
            self._try_match(sid)

    def _log_event(self, sid: str, raw_t, conv_t, delta_ms, matched: bool, reason: str):
        et = now_sec()
        self.csv_writer.writerow([f"{et:.3f}", sid, f"{raw_t:.3f}" if raw_t else "", f"{conv_t:.3f}" if conv_t else "", f"{delta_ms:.2f}" if delta_ms is not None else "", int(matched), reason])
        self.csv_file.flush()

    def on_shutdown(self):
        for sid, rb in self.raw_buf.items():
            while rb:
                r = rb.popleft()
                self._log_event(sid, r, None, None, False, "no_converted_match")

        per_sensor = {}
        overall_attempts = 0
        overall_successes = 0
        for sid, sc in self.sensors.items():
            a = self.attempts[sid]
            s = self.successes[sid]
            sr = (float(s) / a * 100.0) if a > 0 else 0.0
            per_sensor[sid] = {
                "attempts": a,
                "successes": s,
                "success_rate": sr,
                "required": sc.required,
            }
            if sc.required:
                overall_attempts += a
                overall_successes += s

        overall_rate = (float(overall_successes) / overall_attempts * 100.0) if overall_attempts > 0 else 0.0
        summary = {
            "threshold": self.threshold,
            "overall_attempts": overall_attempts,
            "overall_successes": overall_successes,
            "overall_success_rate": overall_rate,
            "per_sensor": per_sensor,
            "pass": overall_rate >= self.threshold
        }
        with open(self.json_path, "w") as f:
            json.dump(summary, f, indent=2)
        rospy.loginfo(f"[validator] Summary written to {self.json_path}")
        rospy.loginfo(f"[validator] CSV written to {self.csv_path}")
        self.csv_file.close()

def main():
    rospy.init_node("conversion_validator")
    ConversionValidator()
    rospy.spin()

if __name__ == "__main__":
    main()
