
#!/usr/bin/env python3
import csv
import json
import os
import threading
import time
from datetime import datetime

import rosbag
import rospy
import yaml
from moveit_msgs.msg import DisplayTrajectory
from rosgraph_msgs.msg import Log


class KPI017Logger:
    def __init__(self):
        cfg_file = rospy.get_param("~config_file", "")
        if not os.path.exists(cfg_file):
            rospy.logerr("Config file not found: %s", cfg_file)
            raise SystemExit

        with open(cfg_file, "r") as f:
            cfg = yaml.safe_load(f)

        base_dir = cfg.get("storage_dir", "/tmp")
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(base_dir, f"kpi_017_{ts}")
        os.makedirs(self.run_dir, exist_ok=True)

        self.csv_path = os.path.join(self.run_dir, "events.csv")
        self.json_path = os.path.join(self.run_dir, "summary.json")
        self.traj_path = os.path.join(self.run_dir, "trajectories.json")
        self.bag_path = os.path.join(self.run_dir, "kpi017.bag")

        self.events = []
        self.before_traj = None
        self.after_traj = None
        self.collision_count = 0
        self.replan_count = 0
        self.last_event_type = None
        self.pass_criteria_met = False
        self.lock = threading.Lock()

        self.bag_proc = threading.Thread(target=self._record_bag)
        self.bag_proc.start()

        rospy.Subscriber("/rosout", Log, self.cb_log, queue_size=50)
        rospy.Subscriber("/ugv0/telehandler/move_group/display_planned_path",
                         DisplayTrajectory, self.cb_traj, queue_size=5)

    def _record_bag(self):
        os.system(
            f"rosbag record -O {self.bag_path} /rosout /ugv0/telehandler/move_group/display_planned_path __name:=kpi017_bag_recorder")

    def cb_log(self, msg: Log):
        txt = msg.msg
        t = time.time()
        if "Predicted collision" in txt or "Collision detected" in txt:
            self.collision_count += 1
            self.last_event_type = "collision"
            self.events.append((t, "collision", txt))
        elif "Replan succeeded" in txt:
            self.replan_count += 1
            self.last_event_type = "replan"
            self.events.append((t, "replan", txt))

        self._check_pass_criteria()

    def cb_traj(self, msg: DisplayTrajectory):
        if self.last_event_type == "collision" and self.before_traj is None:
            self.before_traj = msg
        elif self.last_event_type == "replan":
            self.after_traj = msg

    def _check_pass_criteria(self):
        if self.collision_count >= 1 and self.replan_count >= 1:
            # Check if no collisions happened after last replan
            collisions_after_replan = [e for e in self.events if e[1] == "collision" and e[0] > max(t for t, typ, _ in self.events if typ == "replan")]
            if not collisions_after_replan:
                self.pass_criteria_met = True
                rospy.loginfo("KPI 017 PASS criteria met. Stopping logger...")
                self._finalize()

    def _finalize(self):
        # Stop rosbag recording
        os.system("rosnode kill /kpi017_bag_recorder")
        # Write CSV
        with open(self.csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "event_type", "message"])
            for ts, typ, msg in self.events:
                w.writerow([datetime.fromtimestamp(ts).isoformat(), typ, msg])
        # Write summary JSON
        summary = {
            "collision_count": self.collision_count,
            "replan_count": self.replan_count,
            "pass": self.pass_criteria_met
        }
        with open(self.json_path, "w") as f:
            json.dump(summary, f, indent=2)
        # Write trajectory info
        traj_info = {
            "before": self._traj_metadata(self.before_traj),
            "after": self._traj_metadata(self.after_traj)
        }
        with open(self.traj_path, "w") as f:
            json.dump(traj_info, f, indent=2)
        rospy.signal_shutdown("KPI criteria met")

    def _traj_metadata(self, traj_msg):
        if traj_msg is None or not traj_msg.trajectory:
            return None
        jt = traj_msg.trajectory[0].joint_trajectory
        return {
            "point_count": len(jt.points),
            "duration_s": jt.points[-1].time_from_start.to_sec() if jt.points else 0.0,
            "joint_names": jt.joint_names
        }

if __name__ == "__main__":
    rospy.init_node("kpi_017_logger")
    KPI017Logger()
    rospy.spin()
