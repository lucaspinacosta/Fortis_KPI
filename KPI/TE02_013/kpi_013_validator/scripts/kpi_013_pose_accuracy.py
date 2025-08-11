
#!/usr/bin/env python3
import csv
import json
import math
import os
import time

import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def pose_from_msg(msg):
    if isinstance(msg, Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
    elif isinstance(msg, PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
    else:
        raise TypeError("Unsupported message type: %s" % type(msg))
    return (p.x, p.y, p.z, quat_to_yaw(o))

class KPI013Validator:
    def __init__(self):
        cfg_file = rospy.get_param("~config_file", "")
        if not cfg_file or not os.path.exists(cfg_file):
            rospy.logerr("Config file not found: %s", cfg_file)
            raise SystemExit

        with open(cfg_file, "r") as f:
            cfg = yaml.safe_load(f)
        self.est_topic = cfg["estimated_topic"]
        self.gt_topic = cfg["ground_truth_topic"]
        self.pos_thresh = float(cfg["position_threshold_m"])
        self.log_dir = cfg.get("log_dir", "/tmp/kpi_013_logs")
        os.makedirs(self.log_dir, exist_ok=True)

        ts = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.log_dir, f"kpi013_events_{ts}.csv")
        self.json_path = os.path.join(self.log_dir, f"kpi013_summary_{ts}.json")

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "pos_err_m", "yaw_err_deg", "pass"])

        self.last_est = None
        self.last_gt = None

        rospy.Subscriber(self.est_topic, Odometry, self.cb_est, queue_size=10)
        rospy.Subscriber(self.est_topic, PoseWithCovarianceStamped, self.cb_est, queue_size=10)
        rospy.Subscriber(self.gt_topic, Odometry, self.cb_gt, queue_size=10)
        rospy.Subscriber(self.gt_topic, PoseWithCovarianceStamped, self.cb_gt, queue_size=10)

        self.attempts = 0
        self.successes = 0
        rospy.on_shutdown(self.on_shutdown)

    def cb_est(self, msg): self.last_est = pose_from_msg(msg); self.check()
    def cb_gt(self, msg): self.last_gt = pose_from_msg(msg); self.check()

    def check(self):
        if self.last_est and self.last_gt:
            ex, ey, ez, eyaw = self.last_est
            gx, gy, gz, gyaw = self.last_gt
            pos_err = math.sqrt((ex-gx)**2 + (ey-gy)**2 + (ez-gz)**2)
            yaw_err = abs(eyaw - gyaw) * 180.0 / math.pi
            if yaw_err > 180: yaw_err = 360 - yaw_err
            passed = pos_err <= self.pos_thresh
            self.csv_writer.writerow([time.time(), pos_err, yaw_err, int(passed)])
            self.csv_file.flush()
            self.attempts += 1
            if passed: self.successes += 1
            self.last_est = None
            self.last_gt = None

    def on_shutdown(self):
        summary = {
            "position_threshold_m": self.pos_thresh,
            "attempts": self.attempts,
            "successes": self.successes,
            "success_rate": (self.successes/self.attempts*100.0) if self.attempts else 0.0,
            "pass": self.successes == self.attempts
        }
        with open(self.json_path, "w") as f:
            json.dump(summary, f, indent=2)
        rospy.loginfo("Summary written to %s", self.json_path)
        rospy.loginfo("CSV log written to %s", self.csv_path)
        self.csv_file.close()

if __name__ == "__main__":
    rospy.init_node("kpi_013_pose_accuracy")
    KPI013Validator()
    rospy.spin()
