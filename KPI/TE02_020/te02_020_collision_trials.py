#!/usr/bin/env python3
"""TE02_020 collision avoidance validation runner.

This script runs repeatable MoveIt collision-avoidance trials for KPI TE02_020:

    Collision avoidance success rate > 95% with response time < 250 ms

Each trial:
  1. Clears prior KPI collision objects.
  2. Applies one synthetic obstacle to the MoveIt planning scene.
  3. Measures collision-check response with /check_state_validity.
  4. Requests a plan for a named SRDF group state.
  5. Checks every returned trajectory waypoint with /check_state_validity.

The default obstacle positions are intentionally exposed as CLI options. They are
starting points for a controlled validation and should be tuned if the planner can
solve all trials trivially or if all trials are fully blocked.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, Constraints, DisplayTrajectory, JointConstraint, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray


KPI_DIR = Path(str(os.environ.get('HOME')) +
               "/criarte_ws/scripts/Fortis_KPI/KPI/TE02_020")
SRDF_PATH = Path(str(os.environ.get('HOME')) +
                 "/criarte_ws/src/sw_system/telehandler_moveit/config/robot.srdf")


@dataclass(frozen=True)
class ObstacleSpec:
    name: str
    frame_id: str
    size_xyz: Tuple[float, float, float]
    position_xyz: Tuple[float, float, float]


DEFAULT_OBSTACLES = [
    ObstacleSpec("blocking_front", "base_link", (3.9, 7.9, 5.2), (9.2, 0.0, 0.8)),
    ObstacleSpec("near_left", "base_link", (3.8, 7.8, 5.0), (9.6, 0.85, 0.7)),
    ObstacleSpec("near_right", "base_link", (3.8, 7.8, 5.0), (9.6, -0.85, 0.7)),
    ObstacleSpec("high_block", "base_link", (3.1, 7.0, 5.9), (9.5, 0.0, 1.55)),
]

BASELINE_OBSTACLE = ObstacleSpec("no_obstacle", "base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (pct / 100.0) * (len(ordered) - 1)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[int(rank)]
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def parse_srdf_group_states(srdf_path: Path, group_name: str) -> Dict[str, Dict[str, float]]:
    root = ET.parse(srdf_path).getroot()
    states: Dict[str, Dict[str, float]] = {}
    for group_state in root.findall("group_state"):
        if group_state.attrib.get("group") != group_name:
            continue
        state_name = group_state.attrib["name"]
        joints: Dict[str, float] = {}
        for joint in group_state.findall("joint"):
            joints[joint.attrib["name"]] = float(joint.attrib["value"])
        states[state_name] = joints
    return states


def write_csv(path: Path, rows: Sequence[dict], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class TE02020CollisionTrials(Node):
    def __init__(self, args: argparse.Namespace, obstacles: Sequence[ObstacleSpec], targets: Dict[str, Dict[str, float]]):
        super().__init__("te02_020_collision_trials")
        self.args = args
        self.obstacles = list(obstacles)
        self.targets = targets
        self.latest_joint_state: Optional[JointState] = None
        self.trial_event_pub = self.create_publisher(String, "/te02_020/trial_event", 10)
        self.trial_result_pub = self.create_publisher(String, "/te02_020/trial_result", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/te02_020/markers", 10)
        self.baseline_path_pub = self.create_publisher(DisplayTrajectory, "/te02_020/baseline_display_path", 10)
        self.avoidance_path_pub = self.create_publisher(DisplayTrajectory, "/te02_020/avoidance_display_path", 10)
        self.move_group_display_path_pub = self.create_publisher(DisplayTrajectory, "/move_group/display_planned_path", 10)
        self.create_subscription(JointState, args.joint_states_topic, self._on_joint_state, 50)
        self.apply_scene_client = self.create_client(ApplyPlanningScene, args.apply_planning_scene_service)
        self.plan_client = self.create_client(GetMotionPlan, args.plan_service)
        self.state_validity_client = self.create_client(GetStateValidity, args.state_validity_service)

    def _on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def wait_ready(self) -> bool:
        deadline = time.monotonic() + self.args.service_timeout_s
        clients = [self.apply_scene_client, self.plan_client, self.state_validity_client]
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_joint_state is not None and all(c.service_is_ready() for c in clients):
                return True
        return False

    def _call_service(self, client, request, timeout_s: float):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        if not future.done():
            return None
        return future.result()

    def _publish_event(self, topic_pub, payload: dict) -> None:
        topic_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_markers(self, obstacle: ObstacleSpec, text: str, success: Optional[bool] = None) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        cube = Marker()
        cube.header.frame_id = obstacle.frame_id
        cube.header.stamp = stamp
        cube.ns = "te02_020_obstacle"
        cube.id = 1
        cube.type = Marker.CUBE
        cube.action = Marker.ADD if obstacle.name != "no_obstacle" else Marker.DELETE
        cube.pose.position.x = float(obstacle.position_xyz[0])
        cube.pose.position.y = float(obstacle.position_xyz[1])
        cube.pose.position.z = float(obstacle.position_xyz[2])
        cube.pose.orientation.w = 1.0
        cube.scale.x = max(float(obstacle.size_xyz[0]), 0.01)
        cube.scale.y = max(float(obstacle.size_xyz[1]), 0.01)
        cube.scale.z = max(float(obstacle.size_xyz[2]), 0.01)
        cube.color.r = 1.0
        cube.color.g = 0.25
        cube.color.b = 0.05
        cube.color.a = 0.45
        markers.markers.append(cube)

        label = Marker()
        label.header.frame_id = obstacle.frame_id
        label.header.stamp = stamp
        label.ns = "te02_020_status"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(obstacle.position_xyz[0])
        label.pose.position.y = float(obstacle.position_xyz[1])
        label.pose.position.z = float(obstacle.position_xyz[2]) + max(float(obstacle.size_xyz[2]), 0.5) * 0.75
        label.pose.orientation.w = 1.0
        label.scale.z = 0.25
        label.text = text
        if success is None:
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
        elif success:
            label.color.r = 0.1
            label.color.g = 1.0
            label.color.b = 0.1
        else:
            label.color.r = 1.0
            label.color.g = 0.1
            label.color.b = 0.1
        label.color.a = 1.0
        markers.markers.append(label)
        self.marker_pub.publish(markers)

    def _current_robot_state(self) -> RobotState:
        state = RobotState()
        if self.latest_joint_state is not None:
            state.joint_state.name = list(self.latest_joint_state.name)
            state.joint_state.position = list(self.latest_joint_state.position)
            state.joint_state.velocity = list(self.latest_joint_state.velocity)
            state.joint_state.effort = list(self.latest_joint_state.effort)
        return state

    def _target_robot_state(self, target_joints: Dict[str, float]) -> RobotState:
        state = self._current_robot_state()
        positions = dict(zip(state.joint_state.name, state.joint_state.position))
        positions.update(target_joints)
        state.joint_state.name = list(positions.keys())
        state.joint_state.position = [float(v) for v in positions.values()]
        state.joint_state.velocity = []
        state.joint_state.effort = []
        return state

    def _make_obstacle(self, spec: ObstacleSpec, operation: int) -> CollisionObject:
        obj = CollisionObject()
        obj.header = Header()
        obj.header.frame_id = spec.frame_id
        obj.id = f"te02_020_{spec.name}"
        obj.operation = operation
        if operation == CollisionObject.REMOVE:
            return obj

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(v) for v in spec.size_xyz]
        pose = Pose()
        pose.position.x = float(spec.position_xyz[0])
        pose.position.y = float(spec.position_xyz[1])
        pose.position.z = float(spec.position_xyz[2])
        pose.orientation.w = 1.0
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        return obj

    def _apply_collision_objects(self, objects: Sequence[CollisionObject]) -> Tuple[bool, float]:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = list(objects)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        start = time.monotonic_ns()
        response = self._call_service(self.apply_scene_client, request, self.args.service_timeout_s)
        elapsed_ms = (time.monotonic_ns() - start) / 1e6
        return bool(response and response.success), elapsed_ms

    def clear_obstacles(self) -> Tuple[bool, float]:
        return self._apply_collision_objects([
            self._make_obstacle(spec, CollisionObject.REMOVE) for spec in self.obstacles
        ])

    def add_obstacle(self, spec: ObstacleSpec) -> Tuple[bool, float]:
        return self._apply_collision_objects([self._make_obstacle(spec, CollisionObject.ADD)])

    def _goal_constraints(self, target_joints: Dict[str, float]) -> Constraints:
        constraints = Constraints()
        for name, pos in target_joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = self.args.goal_tolerance
            jc.tolerance_below = self.args.goal_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def request_plan(self, target_name: str) -> Tuple[Optional[object], float, int, str]:
        target_joints = self.targets[target_name]
        request = GetMotionPlan.Request()
        mpr = request.motion_plan_request
        mpr.group_name = self.args.group_name
        mpr.pipeline_id = self.args.pipeline_id
        mpr.planner_id = self.args.planner_id
        mpr.num_planning_attempts = self.args.planning_attempts
        mpr.allowed_planning_time = self.args.allowed_planning_time
        mpr.max_velocity_scaling_factor = self.args.velocity_scaling
        mpr.max_acceleration_scaling_factor = self.args.acceleration_scaling
        mpr.start_state = self._current_robot_state()
        mpr.goal_constraints.append(self._goal_constraints(target_joints))
        if self.args.workspace_frame:
            mpr.workspace_parameters.header.frame_id = self.args.workspace_frame
            mpr.workspace_parameters.min_corner.x = self.args.workspace_min[0]
            mpr.workspace_parameters.min_corner.y = self.args.workspace_min[1]
            mpr.workspace_parameters.min_corner.z = self.args.workspace_min[2]
            mpr.workspace_parameters.max_corner.x = self.args.workspace_max[0]
            mpr.workspace_parameters.max_corner.y = self.args.workspace_max[1]
            mpr.workspace_parameters.max_corner.z = self.args.workspace_max[2]

        start = time.monotonic_ns()
        response = self._call_service(self.plan_client, request, self.args.planning_timeout_s)
        elapsed_ms = (time.monotonic_ns() - start) / 1e6
        if response is None:
            return None, elapsed_ms, -999, "planning_service_timeout"
        code = int(response.motion_plan_response.error_code.val)
        if code != 1:
            return response, elapsed_ms, code, f"planning_failed_error_code_{code}"
        trajectory = response.motion_plan_response.trajectory
        if not trajectory.joint_trajectory.points:
            return response, elapsed_ms, code, "planning_returned_empty_trajectory"
        return response, elapsed_ms, code, "planning_succeeded"

    def publish_display_path(self, plan_response, publisher) -> None:
        if plan_response is None:
            return
        trajectory = plan_response.motion_plan_response.trajectory
        if not trajectory.joint_trajectory.points:
            return
        msg = DisplayTrajectory()
        msg.model_id = "robot"
        msg.trajectory_start = self._current_robot_state()
        msg.trajectory.append(trajectory)
        publisher.publish(msg)

    def _check_waypoint_valid(self, joint_names: Sequence[str], positions: Sequence[float]) -> bool:
        state = self._current_robot_state()
        merged = dict(zip(state.joint_state.name, state.joint_state.position))
        merged.update({name: float(pos) for name, pos in zip(joint_names, positions)})
        state.joint_state.name = list(merged.keys())
        state.joint_state.position = list(merged.values())
        state.joint_state.velocity = []
        state.joint_state.effort = []

        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = self.args.group_name
        response = self._call_service(self.state_validity_client, request, self.args.state_validity_timeout_s)
        return bool(response and response.valid)

    def check_robot_state_valid(self, state: RobotState) -> Tuple[bool, float]:
        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = self.args.group_name
        start = time.monotonic_ns()
        response = self._call_service(self.state_validity_client, request, self.args.state_validity_timeout_s)
        elapsed_ms = (time.monotonic_ns() - start) / 1e6
        return bool(response and response.valid), elapsed_ms

    def validate_trajectory(self, plan_response) -> Tuple[bool, int, int]:
        traj = plan_response.motion_plan_response.trajectory.joint_trajectory
        points = list(traj.points)
        if not points:
            return False, 0, 0
        checked = 0
        invalid = 0
        stride = max(1, self.args.validation_stride)
        selected_indices = set(range(0, len(points), stride))
        selected_indices.add(len(points) - 1)
        for index in sorted(selected_indices):
            checked += 1
            if not self._check_waypoint_valid(traj.joint_names, points[index].positions):
                invalid += 1
        return invalid == 0, checked, invalid

    def run_trial(self, trial_id: int, obstacle: ObstacleSpec, target_name: str) -> dict:
        self._publish_event(self.trial_event_pub, {
            "trial_id": trial_id,
            "event": "start",
            "obstacle": obstacle.name,
            "target": target_name,
            "time_ns": time.time_ns(),
        })
        self._publish_markers(obstacle, f"TE02_020 trial {trial_id}: {obstacle.name} -> {target_name}")

        pre_clear_ok, pre_clear_ms = self.clear_obstacles()
        baseline_response, baseline_planning_ms, baseline_error_code, baseline_status = self.request_plan(target_name)
        if baseline_error_code == 1:
            self.publish_display_path(baseline_response, self.baseline_path_pub)
            self.publish_display_path(baseline_response, self.move_group_display_path_pub)
            self._publish_markers(obstacle, f"TE02_020 trial {trial_id}: baseline path published\n{target_name}")
            if self.args.baseline_display_delay_s > 0.0:
                time.sleep(self.args.baseline_display_delay_s)

        if obstacle.name == "no_obstacle":
            scene_ok = True
            scene_apply_ms = 0.0
        else:
            scene_ok, scene_apply_ms = self.add_obstacle(obstacle)
        candidate_state_valid, detection_check_ms = self.check_robot_state_valid(
            self._target_robot_state(self.targets[target_name])
        )
        detection_response_ms = scene_apply_ms + detection_check_ms
        plan_response, planning_ms, error_code, planning_status = self.request_plan(target_name)
        plan_available_response_ms = scene_apply_ms + planning_ms
        if error_code == 1:
            self.publish_display_path(plan_response, self.avoidance_path_pub)
            self.publish_display_path(plan_response, self.move_group_display_path_pub)
        trajectory_valid = False
        checked_waypoints = 0
        invalid_waypoints = 0
        validation_start_ns = time.monotonic_ns()
        if plan_response is not None and error_code == 1:
            trajectory_valid, checked_waypoints, invalid_waypoints = self.validate_trajectory(plan_response)
        validation_ms = (time.monotonic_ns() - validation_start_ns) / 1e6

        if self.args.response_time_mode == "plan":
            response_time_ms = plan_available_response_ms
        else:
            response_time_ms = detection_response_ms
        if obstacle.name == "no_obstacle":
            post_clear_ok = True
            post_clear_ms = 0.0
        else:
            if self.args.post_trial_display_delay_s > 0.0:
                time.sleep(self.args.post_trial_display_delay_s)
            post_clear_ok, post_clear_ms = self._apply_collision_objects([
                self._make_obstacle(obstacle, CollisionObject.REMOVE)
            ])
        success = bool(scene_ok and error_code == 1 and trajectory_valid and response_time_ms < self.args.response_threshold_ms)
        failure_reasons: List[str] = []
        if self.args.strict_pre_clear and not pre_clear_ok:
            failure_reasons.append("pre_clear_scene_failed")
        if not scene_ok:
            failure_reasons.append("apply_obstacle_failed")
        if error_code != 1:
            failure_reasons.append(planning_status)
        if error_code == 1 and not trajectory_valid:
            failure_reasons.append("trajectory_waypoint_collision")
        if response_time_ms >= self.args.response_threshold_ms:
            failure_reasons.append("response_time_over_threshold")
        if not failure_reasons:
            failure_reasons.append("none")

        row = {
            "trial_id": trial_id,
            "target_name": target_name,
            "group_name": self.args.group_name,
            "obstacle_name": obstacle.name,
            "obstacle_frame": obstacle.frame_id,
            "obstacle_size_xyz": ";".join(str(v) for v in obstacle.size_xyz),
            "obstacle_position_xyz": ";".join(str(v) for v in obstacle.position_xyz),
            "pre_clear_scene_ok": pre_clear_ok,
            "pre_clear_scene_ms": round(pre_clear_ms, 3),
            "baseline_planning_status": baseline_status,
            "baseline_moveit_error_code": baseline_error_code,
            "baseline_planning_response_ms": round(baseline_planning_ms, 3),
            "apply_obstacle_ok": scene_ok,
            "apply_obstacle_ms": round(scene_apply_ms, 3),
            "candidate_state_valid": candidate_state_valid,
            "detection_check_ms": round(detection_check_ms, 3),
            "detection_response_ms": round(detection_response_ms, 3),
            "planning_status": planning_status,
            "moveit_error_code": error_code,
            "planning_response_ms": round(planning_ms, 3),
            "plan_available_response_ms": round(plan_available_response_ms, 3),
            "response_time_ms": round(response_time_ms, 3),
            "validation_ms": round(validation_ms, 3),
            "trajectory_valid": trajectory_valid,
            "checked_waypoints": checked_waypoints,
            "invalid_waypoints": invalid_waypoints,
            "post_clear_scene_ok": post_clear_ok,
            "post_clear_scene_ms": round(post_clear_ms, 3),
            "success": success,
            "failure_reason": ";".join(failure_reasons),
        }
        self._publish_event(self.trial_result_pub, row)
        self._publish_markers(
            obstacle,
            f"TE02_020 trial {trial_id}: {'PASS' if success else 'FAIL'}\n"
            f"response={row['response_time_ms']} ms\n{row['failure_reason']}",
            success=success,
        )
        return row

    def run_all_trials(self) -> List[dict]:
        target_names = self.args.target_states
        rows = []
        for trial_id in range(1, self.args.trials + 1):
            obstacle = self.obstacles[(trial_id - 1) % len(self.obstacles)]
            target_name = target_names[(trial_id - 1) % len(target_names)]
            row = self.run_trial(trial_id, obstacle, target_name)
            rows.append(row)
            self.get_logger().info(
                f"trial={trial_id}/{self.args.trials} target={target_name} obstacle={obstacle.name} "
                f"success={row['success']} response_ms={row['response_time_ms']} reason={row['failure_reason']}"
            )
            time.sleep(self.args.inter_trial_delay_s)
        self.clear_obstacles()
        return rows


def build_summary(rows: Sequence[dict], args: argparse.Namespace) -> List[dict]:
    total = len(rows)
    successes = sum(1 for row in rows if str(row["success"]) == "True" or row["success"] is True)
    success_rate = (successes / total * 100.0) if total else 0.0
    response_times = [float(row["response_time_ms"]) for row in rows]
    over_threshold = sum(1 for value in response_times if value >= args.response_threshold_ms)
    invalid_waypoint_trials = sum(1 for row in rows if int(row["invalid_waypoints"]) > 0)
    planning_successes = sum(1 for row in rows if int(row["moveit_error_code"]) == 1)

    max_response = max(response_times) if response_times else math.nan
    p95_response = percentile(response_times, 95)
    mean_response = statistics.mean(response_times) if response_times else math.nan
    median_response = statistics.median(response_times) if response_times else math.nan

    success_status = "PASS" if success_rate > args.success_threshold_pct else "FAIL"
    max_status = "PASS" if response_times and max_response < args.response_threshold_ms else "FAIL"
    p95_status = "PASS" if response_times and p95_response < args.response_threshold_ms else "FAIL"
    waypoint_status = "PASS" if invalid_waypoint_trials == 0 and total > 0 else "FAIL"
    overall = "PASS" if all(s == "PASS" for s in [success_status, max_status, waypoint_status]) else "FAIL"

    return [
        {"metric": "total_trials", "value": total, "status": "INFO", "details": ""},
        {"metric": "successful_avoidance_trials", "value": successes, "status": success_status, "details": f"threshold>{args.success_threshold_pct}%"},
        {"metric": "avoidance_success_rate_pct", "value": round(success_rate, 3), "status": success_status, "details": f"threshold>{args.success_threshold_pct}%"},
        {"metric": "planning_successful_trials", "value": planning_successes, "status": "INFO", "details": "MoveIt error_code == 1"},
        {"metric": "trajectory_collision_trials", "value": invalid_waypoint_trials, "status": waypoint_status, "details": "invalid waypoints from /check_state_validity"},
        {"metric": "response_time_mode", "value": args.response_time_mode, "status": "INFO", "details": "detection = obstacle apply + /check_state_validity; plan = obstacle apply + /plan_kinematic_path"},
        {"metric": "response_threshold_ms", "value": args.response_threshold_ms, "status": "INFO", "details": ""},
        {"metric": "response_time_mean_ms", "value": round(mean_response, 3) if response_times else "", "status": "INFO", "details": ""},
        {"metric": "response_time_median_ms", "value": round(median_response, 3) if response_times else "", "status": "INFO", "details": ""},
        {"metric": "response_time_p95_ms", "value": round(p95_response, 3) if response_times else "", "status": p95_status, "details": f"threshold<{args.response_threshold_ms} ms"},
        {"metric": "response_time_max_ms", "value": round(max_response, 3) if response_times else "", "status": max_status, "details": f"threshold<{args.response_threshold_ms} ms"},
        {"metric": "response_time_over_threshold_trials", "value": over_threshold, "status": "PASS" if over_threshold == 0 and total > 0 else "FAIL", "details": ""},
        {"metric": "TE02_020_overall", "value": overall, "status": overall, "details": f"{successes}/{total} trials successful"},
    ]


def parse_obstacle_arg(value: str) -> ObstacleSpec:
    # Format: name,frame,x,y,z,sx,sy,sz
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 8:
        raise argparse.ArgumentTypeError("obstacle must be name,frame,x,y,z,sx,sy,sz")
    name, frame = parts[0], parts[1]
    x, y, z, sx, sy, sz = [float(v) for v in parts[2:]]
    return ObstacleSpec(name, frame, (sx, sy, sz), (x, y, z))


def make_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run TE02_020 MoveIt collision avoidance trials.")
    parser.add_argument("--trials", type=int, default=10, help="Number of trials. Use 10 for smoke test, 100 for KPI run.")
    parser.add_argument("--group-name", default="Arm_Group")
    parser.add_argument("--target-states", nargs="+", default=["Pose_Arm_01", "Pose_Arm_02", "Idle2"])
    parser.add_argument("--srdf", type=Path, default=SRDF_PATH)
    parser.add_argument("--output-dir", type=Path, default=KPI_DIR)
    parser.add_argument("--obstacle", action="append", type=parse_obstacle_arg, help="Override/add obstacle: name,frame,x,y,z,sx,sy,sz")
    parser.add_argument("--no-obstacle-baseline", action="store_true", help="Run planning trials without adding obstacles.")
    parser.add_argument("--success-threshold-pct", type=float, default=95.0)
    parser.add_argument("--response-threshold-ms", type=float, default=250.0)
    parser.add_argument("--response-time-mode", choices=["detection", "plan"], default="detection", help="Use collision-check response time or full plan-availability time for KPI response_time_ms.")
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--apply-planning-scene-service", default="/apply_planning_scene")
    parser.add_argument("--plan-service", default="/plan_kinematic_path")
    parser.add_argument("--state-validity-service", default="/check_state_validity")
    parser.add_argument("--pipeline-id", default="ompl")
    parser.add_argument("--planner-id", default="RRTstar")
    parser.add_argument("--planning-attempts", type=int, default=1)
    parser.add_argument("--allowed-planning-time", type=float, default=0.2)
    parser.add_argument("--planning-timeout-s", type=float, default=5.0)
    parser.add_argument("--service-timeout-s", type=float, default=10.0)
    parser.add_argument("--state-validity-timeout-s", type=float, default=1.0)
    parser.add_argument("--goal-tolerance", type=float, default=1e-3)
    parser.add_argument("--velocity-scaling", type=float, default=0.5)
    parser.add_argument("--acceleration-scaling", type=float, default=0.5)
    parser.add_argument("--validation-stride", type=int, default=1, help="Check every Nth trajectory waypoint plus final point.")
    parser.add_argument("--inter-trial-delay-s", type=float, default=0.1)
    parser.add_argument("--baseline-display-delay-s", type=float, default=1.0, help="Pause after publishing the no-obstacle baseline path before applying the obstacle.")
    parser.add_argument("--post-trial-display-delay-s", type=float, default=1.0, help="Pause after publishing the avoidance path/result before clearing the obstacle.")
    parser.add_argument("--strict-pre-clear", action="store_true", help="Fail trials when pre-trial obstacle cleanup reports false.")
    parser.add_argument("--workspace-frame", default="base_link")
    parser.add_argument("--workspace-min", type=float, nargs=3, default=[-8.0, -8.0, -3.0])
    parser.add_argument("--workspace-max", type=float, nargs=3, default=[8.0, 8.0, 8.0])
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = make_arg_parser().parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    obstacles = [BASELINE_OBSTACLE] if args.no_obstacle_baseline else (args.obstacle if args.obstacle else DEFAULT_OBSTACLES)
    states = parse_srdf_group_states(args.srdf, args.group_name)
    missing = [name for name in args.target_states if name not in states]
    if missing:
        raise SystemExit(f"Missing target states for group {args.group_name}: {missing}. Available: {sorted(states)}")
    targets = {name: states[name] for name in args.target_states}

    rclpy.init()
    node = TE02020CollisionTrials(args, obstacles, targets)
    try:
        if not node.wait_ready():
            raise SystemExit(
                "Timed out waiting for /joint_states and MoveIt services. "
                "Check that move_group is running and joint states are available."
            )
        rows = node.run_all_trials()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    trial_csv = args.output_dir / "te02_020_trial_results.csv"
    summary_csv = args.output_dir / "te02_020_summary.csv"
    trial_fields = [
        "trial_id", "target_name", "group_name", "obstacle_name", "obstacle_frame",
        "obstacle_size_xyz", "obstacle_position_xyz", "pre_clear_scene_ok", "pre_clear_scene_ms",
        "baseline_planning_status", "baseline_moveit_error_code", "baseline_planning_response_ms",
        "apply_obstacle_ok", "apply_obstacle_ms", "candidate_state_valid", "detection_check_ms",
        "detection_response_ms", "planning_status", "moveit_error_code", "planning_response_ms",
        "plan_available_response_ms", "response_time_ms", "validation_ms", "trajectory_valid", "checked_waypoints",
        "invalid_waypoints", "post_clear_scene_ok", "post_clear_scene_ms", "success", "failure_reason",
    ]
    write_csv(trial_csv, rows, trial_fields)
    summary_rows = build_summary(rows, args)
    write_csv(summary_csv, summary_rows, ["metric", "value", "status", "details"])
    print(f"Wrote {trial_csv}")
    print(f"Wrote {summary_csv}")
    print(f"TE02_020 overall: {summary_rows[-1]['value']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
