#!/usr/bin/env python3
"""
HIL Trajectory Fidelity Evaluator for 3-Axis Crane System
==========================================================
Offline analysis of a ROS 2 bag file comparing planned vs. actual
joint trajectories. Computes per-joint RMSE and produces diagnostic plots.

Planned trajectory source (in priority order):
  1. --plan CLI argument (recommended for real hardware / action-based goals)
  2. Bag topic: /forward_position_controller/joint_trajectory
  3. Bag topic: action send_goal (if recorded)

Dependencies (no ROS 2 installation needed):
    pip install rosbags numpy scipy matplotlib

Usage:
    # Native units: hook [m], trolley [m], slewing [rad] (see crane README):
    python hil_trajectory_eval.py bag_dir --plan 0,0,0@0 0.5,0.8,1.57@7

    # Slewing goal in degrees; linear joints already in metres:
    python hil_trajectory_eval.py bag_dir --plan 0,0,0@0 0.5,0.8,90@7 --deg-joints slewing_joint

    # All angular waypoints in degrees (only use if you really mean that for every joint):
    python hil_trajectory_eval.py bag_dir --plan 0,0,0@0 180,180,180@2 --degrees

    # Plan already in native units (radians / metres):
    python hil_trajectory_eval.py bag_dir --plan 0,0,0@0 3.14,3.14,3.14@2

    # Per-joint motion anchors (default --align-plan-to first_motion) when axes start at different times.
    # Legacy single anchor: --align-plan-to first_motion_global
    # To use the first bag timestamp instead: --align-plan-to bag_start

    # Waypoints in CAN motor degrees (same conversion as moveit_bridge.cpp):
    # python hil_trajectory_eval.py bag_dir --plan 0,0,0@0 100,200,90@7 --plan-motor-deg

    # If hook/trolley actual moves opposite to the plan, try: --invert-prismatic
    # Or rely on default auto-invert (Pearson r check); disable with --no-auto-invert

    # Linear interpolation instead of cubic (JointTrajectoryController-style):
    python hil_trajectory_eval.py bag_dir --plan 0,0,0@0 180,180,180@2 --interp linear

    # Planned trajectory recorded in the bag (auto-detect):
    python hil_trajectory_eval.py bag_dir
"""

import sys
import argparse
from pathlib import Path

import numpy as np
from scipy.interpolate import interp1d, CubicHermiteSpline
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
JOINT_NAMES = ["hook_joint", "trolley_joint", "slewing_joint"]

# Plain topic (used by topic-based commanding)
TOPIC_PLANNED_PLAIN = "/forward_position_controller/joint_trajectory"
# Action goal topic (used by MoveIt / ros2 action send_goal)
TOPIC_PLANNED_ACTION = (
    "/forward_position_controller/follow_joint_trajectory/_action/send_goal"
)
TOPIC_ACTUAL_DEFAULT = "/joint_states"

# ---------------------------------------------------------------------------
# Must match crane/crane_master/src/moveit_bridge.cpp (motor deg ↔ joint_states)
# ---------------------------------------------------------------------------
TROLLEY_DRIVE_DIAMETER_MM = 24.0
HOIST_DRIVE_DIAMETER_MM = 80.0
# Trolley: (D * pi) / (377.36 * 360) — same magic number as the bridge
METERS_PER_DEGREE_TROLLEY = (TROLLEY_DRIVE_DIAMETER_MM * np.pi) / (377.36 * 360.0)
METERS_PER_DEGREE_HOIST = (HOIST_DRIVE_DIAMETER_MM * np.pi) / (2000.0 * 360.0)


def motor_deg_to_joint_state(jname: str, deg: float) -> float:
    """CAN motor degrees → /joint_states units (m for prismatic, rad for slewing)."""
    if jname == "slewing_joint":
        return np.deg2rad(deg)
    if jname == "hook_joint":
        return deg * METERS_PER_DEGREE_HOIST
    if jname == "trolley_joint":
        return deg * METERS_PER_DEGREE_TROLLEY
    raise KeyError(jname)


def apply_invert_joints(actual: dict, joint_names: set[str]) -> dict:
    """Negate selected joint positions (URDF / motor sign vs MoveIt convention)."""
    out = {}
    for jname in JOINT_NAMES:
        t_arr, p_arr = actual[jname]
        if jname in joint_names:
            out[jname] = (np.copy(t_arr), -np.copy(p_arr))
        else:
            out[jname] = (np.copy(t_arr), np.copy(p_arr))
    return out


def apply_invert_prismatic(actual: dict) -> dict:
    """Negate hook and trolley positions (URDF vs motor sign)."""
    return apply_invert_joints(
        actual, {"hook_joint", "trolley_joint"},
    )


def auto_detect_inverted_joints(results: dict, corr_threshold: float = -0.25) -> set[str]:
    """If Δ plan and Δ act move in opposite directions, suggest negating those joints."""
    out: set[str] = set()
    for jname in JOINT_NAMES:
        r = results.get(jname)
        if r is None:
            continue
        p = r["planned_interp"]
        a = r["actual_pos"]
        if p.size < 3 or np.std(a) < 1e-9 or np.std(p) < 1e-12:
            continue
        c = float(np.corrcoef(p, a)[0, 1])
        if np.isnan(c):
            continue
        if c < corr_threshold:
            out.add(jname)
    return out


def print_correlation_summary(results: dict) -> None:
    """Pearson r between planned and actual (same series as plots)."""
    print("\n[INFO] Pearson r (Δ plan vs Δ act) in overlap:")
    for jname in JOINT_NAMES:
        r = results.get(jname)
        if r is None:
            print(f"         {jname:16s}  —")
            continue
        p, a = r["planned_interp"], r["actual_pos"]
        if p.size < 2 or np.std(a) < 1e-12:
            print(f"         {jname:16s}  n/a")
            continue
        c = float(np.corrcoef(p, a)[0, 1])
        lab = f"{c: .4f}" if not np.isnan(c) else "n/a"
        print(f"         {jname:16s}  r = {lab}")


def stamp_to_sec(stamp) -> float:
    """Convert a builtin_interfaces/Time stamp to float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def duration_to_sec(dur) -> float:
    """Convert a builtin_interfaces/Duration to float seconds."""
    return dur.sec + dur.nanosec * 1e-9


# ---------------------------------------------------------------------------
# Type registration — control_msgs is not in the default Humble store
# ---------------------------------------------------------------------------
def _register_control_msgs(typestore):
    """Register the control_msgs action types needed to deserialise
    FollowJointTrajectory_SendGoal_Request messages."""

    # 1. JointTolerance
    add_types = get_types_from_msg(
        "string name\nfloat64 position\nfloat64 velocity\nfloat64 acceleration",
        "control_msgs/msg/JointTolerance",
    )
    typestore.register(add_types)

    # 2. FollowJointTrajectory_Goal  (the Goal part of the action)
    add_types = get_types_from_msg(
        "trajectory_msgs/msg/JointTrajectory trajectory\n"
        "control_msgs/msg/JointTolerance[] path_tolerance\n"
        "control_msgs/msg/JointTolerance[] goal_tolerance\n"
        "builtin_interfaces/msg/Duration goal_time_tolerance",
        "control_msgs/action/FollowJointTrajectory_Goal",
    )
    typestore.register(add_types)

    # 3. UUID (unique_identifier_msgs) — may already exist; register if not
    if "unique_identifier_msgs/msg/UUID" not in typestore.types:
        add_types = get_types_from_msg(
            "uint8[16] uuid",
            "unique_identifier_msgs/msg/UUID",
        )
        typestore.register(add_types)

    # 4. FollowJointTrajectory_SendGoal_Request  (what the bag records)
    add_types = get_types_from_msg(
        "unique_identifier_msgs/msg/UUID goal_id\n"
        "control_msgs/action/FollowJointTrajectory_Goal goal",
        "control_msgs/action/FollowJointTrajectory_SendGoal_Request",
    )
    typestore.register(add_types)


# ---------------------------------------------------------------------------
# Bag parsing
# ---------------------------------------------------------------------------
def extract_actual(bag_path: str, actual_topic: str = TOPIC_ACTUAL_DEFAULT):
    """Extract actual joint positions from the selected JointState topic in the bag.

    Returns
    -------
    actual : dict[str, (np.ndarray, np.ndarray)]
        {joint_name: (times, positions)}
    info : dict
        ``topic_msg_count`` — messages deserialized from *actual_topic*
        ``sample_joint_names`` — joint ``name`` list from the first such message, if any
    """
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    actual_raw = {j: [] for j in JOINT_NAMES}
    info = {"topic_msg_count": 0, "sample_joint_names": None}

    with Reader(bag_path) as reader:
        bag_topics = {conn.topic: conn.msgcount for conn in reader.connections}
        print("[INFO] Topics in bag:")
        for topic, count in bag_topics.items():
            print(f"         {topic}  ({count} msgs)")

        if actual_topic not in bag_topics:
            print(
                f"\n[ERROR] Topic '{actual_topic}' is not in this bag. "
                "Choose a topic from the list above (e.g. /joint_states)."
            )
            actual = {j: (np.array([]), np.array([])) for j in JOINT_NAMES}
            return actual, info

        for conn, timestamp, rawdata in reader.messages():
            if conn.topic == actual_topic:
                info["topic_msg_count"] += 1
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                if info["sample_joint_names"] is None:
                    try:
                        info["sample_joint_names"] = list(msg.name)
                    except AttributeError:
                        info["sample_joint_names"] = []
                t = stamp_to_sec(msg.header.stamp)
                try:
                    name_to_idx = {
                        name: idx for idx, name in enumerate(msg.name)
                    }
                except AttributeError:
                    continue
                for jname in JOINT_NAMES:
                    if jname in name_to_idx:
                        idx = name_to_idx[jname]
                        actual_raw[jname].append((t, msg.position[idx]))

    actual = {}
    for jname in JOINT_NAMES:
        pairs = actual_raw[jname]
        if not pairs:
            actual[jname] = (np.array([]), np.array([]))
            continue
        pairs.sort(key=lambda p: p[0])
        arr = np.array(pairs)
        actual[jname] = (arr[:, 0], arr[:, 1])
    return actual, info


def print_actual_summary(actual: dict) -> None:
    """Print min/max/mean of measured positions so unit/scale issues are obvious."""
    print("\n[INFO] Actual position summary (same units as in the bag / URDF):")
    for jname in JOINT_NAMES:
        t_arr, p_arr = actual[jname]
        if t_arr.size == 0:
            print(f"         {jname:16s}  no samples (joint missing from topic?)")
            continue
        print(
            f"         {jname:16s}  n={t_arr.size:6d}  "
            f"min={np.min(p_arr): .6g}  max={np.max(p_arr): .6g}  "
            f"mean={np.mean(p_arr): .6g}"
        )


def warn_scale_and_units(planned: dict, results: dict) -> None:
    """Warn when planned magnitude dwarfs actual — usually wrong --plan units."""
    # hook/trolley are prismatic (metres); slewing is revolute (radians).
    linear = {"hook_joint", "trolley_joint"}
    for jname in JOINT_NAMES:
        res = results.get(jname)
        if res is None:
            continue
        act = res["actual_pos"]
        plan = res["planned_interp"]
        a_max = float(np.max(np.abs(act))) if act.size else 0.0
        p_max = float(np.max(np.abs(plan))) if plan.size else 0.0
        if a_max < 1e-9 and p_max > 1e-3:
            if res.get("displacement"):
                print(
                    f"[INFO] '{jname}': Δ actual ~0 over overlap — no motion in this window "
                    "(e.g. joint still idle, already at goal, or step faster than sample rate)."
                )
            else:
                print(
                    f"[WARN] '{jname}': actual is ~0 but planned is not — "
                    "missing feedback on this topic, or wrong joint name in the bag."
                )
            continue
        if not res.get("displacement"):
            if jname in linear and p_max > 5.0 and a_max > 1e-9 and (a_max / p_max) < 0.05:
                print(
                    f"[WARN] '{jname}': planned peak (~{p_max:.3g}) >> actual (~{a_max:.3g}). "
                    "For prismatic joints, --plan positions must be in **metres**, not degrees. "
                    "Example: 0.8 m travel, not 180."
                )
            if jname == "slewing_joint" and p_max > 2 * np.pi + 0.5 and a_max > 1e-9 and (a_max / p_max) < 0.05:
                print(
                    f"[WARN] '{jname}': planned peak (~{p_max:.3g} rad) >> actual (~{a_max:.3g}). "
                    "Use radians in --plan, or pass --deg-joints slewing_joint if waypoints are degrees."
                )


def extract_planned_from_bag(bag_path: str, actual):
    """Try to extract planned trajectory from topics in the bag.

    Returns
    -------
    planned : dict[str, (np.ndarray, np.ndarray)] or None
        None if no planned data found in bag.
    """
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    _register_control_msgs(typestore)

    # Build a lookup interpolator from actual data for start-point synthesis
    actual_interp = {}
    for jname in JOINT_NAMES:
        t_arr, p_arr = actual[jname]
        if t_arr.size >= 2:
            actual_interp[jname] = interp1d(
                t_arr, p_arr, kind="nearest", bounds_error=False,
                fill_value=(p_arr[0], p_arr[-1]),
            )

    planned_raw = {j: [] for j in JOINT_NAMES}

    with Reader(bag_path) as reader:
        for conn, timestamp, rawdata in reader.messages():
            traj = None
            t_base = None

            if conn.topic == TOPIC_PLANNED_PLAIN:
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                traj = msg
                t_header = stamp_to_sec(traj.header.stamp)
                t_base = t_header if t_header > 1.0 else timestamp * 1e-9

            elif conn.topic == TOPIC_PLANNED_ACTION:
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                traj = msg.goal.trajectory
                t_header = stamp_to_sec(traj.header.stamp)
                t_base = t_header if t_header > 1.0 else timestamp * 1e-9

            if traj is None:
                continue

            try:
                name_to_idx = {
                    name: idx for idx, name in enumerate(traj.joint_names)
                }
            except AttributeError:
                continue

            has_start = any(
                duration_to_sec(pt.time_from_start) < 1e-6
                for pt in traj.points
            )
            if not has_start:
                for jname in JOINT_NAMES:
                    if jname in name_to_idx and jname in actual_interp:
                        start_pos = float(actual_interp[jname](t_base))
                        planned_raw[jname].append((t_base, start_pos))

            for point in traj.points:
                t_abs = t_base + duration_to_sec(point.time_from_start)
                for jname in JOINT_NAMES:
                    if jname in name_to_idx:
                        idx = name_to_idx[jname]
                        planned_raw[jname].append(
                            (t_abs, point.positions[idx])
                        )

    # Check if we found any planned data
    if all(len(v) == 0 for v in planned_raw.values()):
        return None

    planned = {}
    for jname in JOINT_NAMES:
        pairs = planned_raw[jname]
        if not pairs:
            planned[jname] = (np.array([]), np.array([]))
            continue
        pairs.sort(key=lambda p: p[0])
        arr = np.array(pairs)
        planned[jname] = (arr[:, 0], arr[:, 1])
    return planned


def compute_plan_time_anchors(
    actual: dict,
    align: str,
    motion_threshold: float,
    plan_offset_sec: float,
    verbose: bool = True,
) -> dict[str, float]:
    """Absolute ROS time for each joint's first --plan waypoint (relative time 0).

    For *first_motion*, each joint gets its **own** anchor at the first sample
    where that joint moves by more than *motion_threshold*.  A single global
    anchor (see *first_motion_global*) mis-aligns axes that start at different
    times: the 7.2 s window can sit entirely before trolley motion or after
    slewing has already reached its goal, producing flat Δ actual traces.
    """
    off = float(plan_offset_sec)
    starts = [actual[j][0][0] for j in JOINT_NAMES if actual[j][0].size > 0]
    if not starts:
        return {j: 0.0 + off for j in JOINT_NAMES}
    t_bag_start = float(min(starts))

    if align == "bag_start":
        return {j: t_bag_start + off for j in JOINT_NAMES}

    def _first_motion_time(jname: str) -> float:
        t_arr, p_arr = actual[jname]
        if t_arr.size < 2:
            return float(t_arr[0]) if t_arr.size else t_bag_start
        p0 = float(p_arr[0])
        for i in range(1, len(p_arr)):
            if abs(float(p_arr[i]) - p0) > motion_threshold:
                return float(t_arr[i])
        return float(t_arr[0])

    if align == "first_motion_global":
        motion_times = [_first_motion_time(j) for j in JOINT_NAMES]
        t0 = min(motion_times)
        if verbose:
            print(
                f"[INFO] align-plan-to first_motion_global: single anchor {t0:.6f} "
                f"(+{t0 - t_bag_start:.3f} s from first sample) — "
                "prefer 'first_motion' if axes move at different times."
            )
        return {j: t0 + off for j in JOINT_NAMES}

    if align == "first_motion":
        out = {j: _first_motion_time(j) + off for j in JOINT_NAMES}
        if verbose:
            print("[INFO] align-plan-to first_motion (per-joint anchors):")
            for jname in JOINT_NAMES:
                tj = out[jname] - off
                print(
                    f"         {jname:16s}  {out[jname]:.6f}  "
                    f"(Δ{tj - t_bag_start:+.3f} s from first sample)"
                )
        return out

    return {j: t_bag_start + off for j in JOINT_NAMES}


def auto_scale_plan_duration(
    planned: dict,
    actual: dict,
    motion_threshold: float = 0.02,
    settle_frac: float = 0.95,
    verbose: bool = True,
) -> dict:
    """Scale each joint's plan time so the plan duration matches the actual forward-motion duration.

    For each joint the function detects:
    - **motion start**: first sample that departs from the initial position by
      more than *motion_threshold*.
    - **motion end**: first sample whose displacement (from start) reaches
      *settle_frac* (default 95 %) of the peak displacement **in the same
      direction as the plan**.  This avoids counting a return leg or idle
      tail as part of the forward motion.
    """
    scaled = {}
    for jname in JOINT_NAMES:
        t_plan, pos_plan = planned[jname]
        t_act, pos_act = actual[jname]

        if t_plan.size < 2 or t_act.size < 2:
            scaled[jname] = (np.copy(t_plan), np.copy(pos_plan))
            continue

        # Direction of the planned motion
        plan_disp = float(pos_plan[-1] - pos_plan[0])
        if abs(plan_disp) < 1e-9:
            scaled[jname] = (np.copy(t_plan), np.copy(pos_plan))
            continue

        # Detect actual motion start
        p0 = float(pos_act[0])
        t_motion_start = float(t_act[0])
        for i in range(1, len(pos_act)):
            if abs(float(pos_act[i]) - p0) > motion_threshold:
                t_motion_start = float(t_act[i])
                break

        # Displacement from start, same sign convention as the plan
        disp = pos_act - p0
        if plan_disp > 0:
            peak_disp = float(np.max(disp))
        else:
            peak_disp = float(np.min(disp))

        if abs(peak_disp) < 1e-9:
            scaled[jname] = (np.copy(t_plan), np.copy(pos_plan))
            continue

        # Motion end: first sample reaching settle_frac of peak displacement
        target = settle_frac * peak_disp
        t_motion_end = float(t_act[-1])
        for i in range(len(disp)):
            if plan_disp > 0 and float(disp[i]) >= target:
                t_motion_end = float(t_act[i])
                break
            elif plan_disp < 0 and float(disp[i]) <= target:
                t_motion_end = float(t_act[i])
                break

        actual_duration = t_motion_end - t_motion_start
        plan_duration = float(t_plan[-1] - t_plan[0])

        if plan_duration < 1e-6 or actual_duration < 1e-6:
            scaled[jname] = (np.copy(t_plan), np.copy(pos_plan))
            continue

        scale = actual_duration / plan_duration
        t_scaled = t_plan[0] + (t_plan - t_plan[0]) * scale
        scaled[jname] = (t_scaled, np.copy(pos_plan))

        if verbose:
            print(
                f"[INFO] auto-duration {jname}: "
                f"{plan_duration:.1f}s → {actual_duration:.1f}s (×{scale:.2f})"
            )

    return scaled


def auto_align_time_offset(
    planned: dict,
    actual: dict,
    interp_kind: str = "linear",
    displacement: bool = True,
    max_shift: float = 3.0,
    step: float = 0.05,
    verbose: bool = True,
) -> dict:
    """Shift each joint's plan time by the offset that minimises RMSE.

    A small grid search (±\ *max_shift* in *step* increments) is performed
    independently per joint after duration scaling has been applied.
    """
    aligned = {}
    for jname in JOINT_NAMES:
        t_plan, pos_plan = planned[jname]
        t_act, pos_act = actual[jname]

        if t_plan.size < 2 or t_act.size < 2:
            aligned[jname] = (np.copy(t_plan), np.copy(pos_plan))
            continue

        best_rmse = float("inf")
        best_shift = 0.0

        for shift in np.arange(-max_shift, max_shift + step / 2, step):
            t_shifted = t_plan + shift
            t_start = max(float(t_shifted[0]), float(t_act[0]))
            t_end = min(float(t_shifted[-1]), float(t_act[-1]))
            if t_start >= t_end:
                continue

            mask = (t_act >= t_start) & (t_act <= t_end)
            t_trim = t_act[mask]
            act_trim = pos_act[mask]
            if t_trim.size < 3:
                continue

            fn = interp1d(
                t_shifted, pos_plan, kind=interp_kind,
                bounds_error=False, fill_value="extrapolate",
            )
            plan_interp = fn(t_trim)

            if displacement and plan_interp.size > 0:
                plan_interp = plan_interp - plan_interp[0]
                act_trim = act_trim - act_trim[0]

            rmse = float(np.sqrt(np.mean((plan_interp - act_trim) ** 2)))
            if rmse < best_rmse:
                best_rmse = rmse
                best_shift = float(shift)

        aligned[jname] = (t_plan + best_shift, np.copy(pos_plan))
        if verbose:
            print(
                f"[INFO] auto-align {jname}: shift {best_shift:+.2f}s "
                f"(RMSE {best_rmse:.4f})"
            )

    return aligned


def build_planned_from_cli(
    plan_args: list[str],
    actual,
    degrees: bool = False,
    deg_joints: list[str] | None = None,
    t_anchor: float | dict[str, float] | None = None,
    plan_in_motor_deg: bool = False,
) -> dict:
    """Build a planned trajectory from --plan CLI waypoints.

    Each waypoint is formatted as:
        hook_pos,trolley_pos,slewing_pos@time_sec

    Example:
        --plan 0,0,0@0  180,180,180@2

    Waypoint relative time 0 is placed at *t_anchor* (ROS time), usually the
    first ``joint_states`` sample or the time of first motion; see
    :func:`compute_plan_time_anchors`.

    Parameters
    ----------
    degrees : bool
        If True, convert ALL waypoint positions from degrees to radians.
    deg_joints : list[str] or None
        If given, only the listed joints are converted from degrees to
        radians.  Takes precedence over *degrees*.
    t_anchor : float or None
        Absolute time (seconds) for waypoint ``@0``.  If None, uses the
        earliest timestamp among actual joint series (bag start).

    Returns
    -------
    planned : dict[str, (np.ndarray, np.ndarray)]
    """
    # Build set of joint indices that need deg→rad conversion (MoveIt slewing only)
    deg_indices: set[int] = set()
    if not plan_in_motor_deg:
        if deg_joints:
            for jname in deg_joints:
                if jname not in JOINT_NAMES:
                    print(f"[WARN] --deg-joints: unknown joint '{jname}' (known: {JOINT_NAMES})")
                else:
                    deg_indices.add(JOINT_NAMES.index(jname))
        elif degrees:
            deg_indices = set(range(len(JOINT_NAMES)))

    waypoints = []  # list of (t_sec, [pos_hook, pos_trolley, pos_slewing])
    for token in plan_args:
        if "@" not in token:
            print(f"[ERROR] Invalid waypoint format '{token}'. Expected: pos1,pos2,pos3@time")
            sys.exit(1)
        pos_str, t_str = token.rsplit("@", 1)
        positions = [float(x) for x in pos_str.split(",")]
        if len(positions) != len(JOINT_NAMES):
            print(f"[ERROR] Expected {len(JOINT_NAMES)} positions, got {len(positions)} in '{token}'")
            sys.exit(1)
        if plan_in_motor_deg:
            for ji, jname in enumerate(JOINT_NAMES):
                positions[ji] = motor_deg_to_joint_state(jname, positions[ji])
        else:
            for idx in deg_indices:
                positions[idx] = np.deg2rad(positions[idx])
        waypoints.append((float(t_str), positions))

    waypoints.sort(key=lambda w: w[0])

    if t_anchor is None:
        starts = [actual[j][0][0] for j in JOINT_NAMES if actual[j][0].size > 0]
        if starts:
            t_anchor = float(min(starts))
        else:
            print(
                "[WARN] No actual joint samples — cannot align plan to bag time; "
                "using t_anchor = 0.0. Fix --actual-topic or re-record."
            )
            t_anchor = 0.0

    planned = {}
    for ji, jname in enumerate(JOINT_NAMES):
        if isinstance(t_anchor, dict):
            t0j = float(t_anchor[jname])
        else:
            t0j = float(t_anchor)
        times = np.array([t0j + w[0] for w in waypoints])
        positions = np.array([w[1][ji] for w in waypoints])
        planned[jname] = (times, positions)

    return planned


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def analyse(
    planned,
    actual,
    interp_kind="cubic",
    displacement: bool = True,
    verbose: bool = True,
):
    """
    For each joint:
      1. Determine the overlapping time window.
      2. Normalise time so t=0 is the start of the overlap.
      3. Interpolate sparse planned data onto the actual timestamps.
      4. Compute the tracking error and RMSE.

    Parameters
    ----------
    interp_kind : str
        ``"linear"`` – piecewise-linear interpolation of the plan.
        ``"cubic"``  – cubic Hermite spline with zero velocity at every
                       waypoint.  This matches the default behaviour of
                       ``JointTrajectoryController`` when only positions
                       (no velocities) are given in the goal.

    Returns
    -------
    results : dict[str, dict]
        Per-joint dict with keys:
            t           – normalised time array (aligned to actual timestamps)
            planned_interp – interpolated planned positions
            actual_pos  – actual positions (trimmed to overlap)
            error       – tracking error (planned - actual)
            rmse        – scalar RMSE value
    """
    results = {}

    for jname in JOINT_NAMES:
        t_plan, pos_plan = planned[jname]
        t_act,  pos_act  = actual[jname]

        if t_plan.size < 2 or t_act.size < 2:
            if verbose:
                print(f"[WARN] Insufficient data for '{jname}' — skipping.")
            results[jname] = None
            continue

        # --- Overlapping time window ---
        t_start = max(t_plan[0], t_act[0])
        t_end   = min(t_plan[-1], t_act[-1])

        if t_start >= t_end:
            if verbose:
                print(f"[WARN] No overlapping window for '{jname}' — skipping.")
            results[jname] = None
            continue

        # --- Trim actual data to the overlap ---
        mask = (t_act >= t_start) & (t_act <= t_end)
        t_act_trim   = t_act[mask]
        pos_act_trim = pos_act[mask]

        # --- Interpolate planned trajectory onto actual timestamps ---
        if interp_kind == "cubic" and t_plan.size >= 2:
            # Cubic Hermite spline with zero velocity at every waypoint.
            # JointTrajectoryController uses this profile when only
            # positions (no velocities) are specified in the goal.
            interp_fn = CubicHermiteSpline(
                t_plan, pos_plan, dydx=np.zeros_like(pos_plan),
            )
        else:
            interp_fn = interp1d(
                t_plan, pos_plan,
                kind="linear",
                bounds_error=False,
                fill_value="extrapolate",
            )
        pos_plan_interp = interp_fn(t_act_trim)

        if displacement and pos_plan_interp.size > 0:
            pos_plan_interp = pos_plan_interp - pos_plan_interp[0]
            pos_act_trim = pos_act_trim - pos_act_trim[0]

        # --- Normalise time so overlap starts at t = 0 ---
        t_norm = t_act_trim - t_start

        # --- Tracking error & RMSE ---
        error = pos_plan_interp - pos_act_trim
        rmse  = np.sqrt(np.mean(error ** 2))

        results[jname] = {
            "t": t_norm,
            "planned_interp": pos_plan_interp,
            "actual_pos": pos_act_trim,
            "error": error,
            "rmse": rmse,
            "displacement": displacement,
        }

    return results


def _results_total_rmse(results: dict) -> float | None:
    vals = [results[j]["rmse"] for j in JOINT_NAMES if results.get(j) is not None]
    if len(vals) != len(JOINT_NAMES):
        return None
    return float(sum(vals))


def _results_min_motion_span(results: dict) -> float:
    spans = []
    for jname in JOINT_NAMES:
        res = results.get(jname)
        if res is None:
            return 0.0
        pos = res["actual_pos"]
        if pos.size == 0:
            return 0.0
        spans.append(float(np.max(pos) - np.min(pos)))
    return float(min(spans)) if spans else 0.0


def find_best_fit_configuration(base_actual: dict, args) -> tuple[dict, dict, dict, set[str], dict]:
    """Search a small parameter grid for the lowest-RMSE honest fit."""
    align_options = ["bag_start", "first_motion", "first_motion_global"]
    threshold_options = [0.005, 0.01, 0.02, 0.03, 0.05]
    offset_options = np.arange(-8.0, 8.01, 0.25)
    displacement_options = [True, False]
    interp_options = ["linear", "cubic"]
    best = None

    for align in align_options:
        for motion_threshold in threshold_options:
            for plan_offset_sec in offset_options:
                t_anchor = compute_plan_time_anchors(
                    base_actual,
                    align=align,
                    motion_threshold=motion_threshold,
                    plan_offset_sec=float(plan_offset_sec),
                    verbose=False,
                )
                planned = build_planned_from_cli(
                    args.plan,
                    base_actual,
                    degrees=args.degrees,
                    deg_joints=args.deg_joints,
                    t_anchor=t_anchor,
                    plan_in_motor_deg=args.plan_motor_deg,
                )
                for displacement in displacement_options:
                    for interp_kind in interp_options:
                        res0 = analyse(
                            planned,
                            base_actual,
                            interp_kind=interp_kind,
                            displacement=displacement,
                            verbose=False,
                        )
                        if args.no_auto_invert:
                            invert_set = set()
                        else:
                            invert_set = auto_detect_inverted_joints(res0)
                        actual = apply_invert_joints(base_actual, invert_set)
                        results = analyse(
                            planned,
                            actual,
                            interp_kind=interp_kind,
                            displacement=displacement,
                            verbose=False,
                        )
                        total_rmse = _results_total_rmse(results)
                        if total_rmse is None:
                            continue
                        min_motion_span = _results_min_motion_span(results)
                        if min_motion_span < args.best_fit_min_motion:
                            continue
                        item = {
                            "score": total_rmse,
                            "min_motion_span": min_motion_span,
                            "align": align,
                            "motion_threshold": motion_threshold,
                            "plan_offset_sec": float(plan_offset_sec),
                            "interp": interp_kind,
                            "displacement": displacement,
                            "invert_set": invert_set,
                            "planned": planned,
                            "actual": actual,
                            "results": results,
                        }
                        if best is None or item["score"] < best["score"]:
                            best = item

    if best is None:
        print(
            "[ERROR] --best-fit could not find a valid moving overlap. "
            "Try lowering --best-fit-min-motion or checking the bag/topic."
        )
        sys.exit(1)

    print("[INFO] --best-fit selected:")
    print(
        f"         align={best['align']}  motion-threshold={best['motion_threshold']:.3g}  "
        f"plan-offset-sec={best['plan_offset_sec']:.3f}"
    )
    print(
        f"         interp={best['interp']}  "
        f"mode={'displacement' if best['displacement'] else 'absolute'}  "
        f"invert={sorted(best['invert_set'])}"
    )
    print(
        f"         total RMSE={best['score']:.6f}  "
        f"min motion span={best['min_motion_span']:.6f}"
    )
    return (
        best["planned"],
        best["actual"],
        best["results"],
        set(best["invert_set"]),
        best,
    )


# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------

# Joint display names and units for axis labels
_JOINT_LABELS = {
    "hook_joint":    ("Hook (Hoist)",   "m"),
    "trolley_joint": ("Trolley",        "m"),
    "slewing_joint": ("Slewing",        "rad"),
}

_COLOR_PLAN   = "#2563EB"   # blue-600
_COLOR_ACTUAL = "#F97316"   # orange-500
_COLOR_ERROR  = "#DC2626"   # red-600
_COLOR_FILL   = "#FECACA"   # red-200
_COLOR_GRID   = "#E5E7EB"   # gray-200
_COLOR_RMSE_BG = "#FEF9C3"  # yellow-100
_COLOR_RMSE_BD = "#CA8A04"  # yellow-600


def plot_results(results, step_actual: bool = True):
    """
    Create a 3×2 subplot grid:
        Left column  — Planned vs Actual trajectory per joint
        Right column — Tracking error per joint

    step_actual
        Draw measured series with steps-post (holds each sample until the next),
        matching discrete joint_states / encoder updates.
    """
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "figure.facecolor": "white",
        "axes.facecolor": "#FAFAFA",
        "axes.edgecolor": "#D1D5DB",
        "axes.linewidth": 0.8,
        "xtick.color": "#6B7280",
        "ytick.color": "#6B7280",
        "axes.labelcolor": "#374151",
        "axes.titlecolor": "#111827",
    })

    fig, axes = plt.subplots(
        nrows=3, ncols=2,
        figsize=(15, 10),
        constrained_layout=True,
    )
    fig.suptitle(
        "HIL Trajectory Fidelity — 3-Axis Crane",
        fontsize=15, fontweight="bold", color="#111827", y=0.98,
    )

    for row, jname in enumerate(JOINT_NAMES):
        ax_traj = axes[row, 0]
        ax_err  = axes[row, 1]
        label, unit = _JOINT_LABELS.get(jname, (jname, ""))

        res = results.get(jname)
        if res is None:
            for ax in (ax_traj, ax_err):
                ax.text(0.5, 0.5, "No data", ha="center", va="center",
                        transform=ax.transAxes, fontsize=11, color="#9CA3AF")
                ax.set_yticks([])
            ax_traj.set_ylabel(label, fontweight="bold")
            continue

        t = res["t"]

        # ----- Left: trajectory tracking -----
        ax_traj.plot(
            t, res["planned_interp"],
            color=_COLOR_PLAN, linewidth=2.0, label="Planned", zorder=3,
        )
        act_style = {"drawstyle": "steps-post"} if step_actual else {}
        ax_traj.plot(
            t, res["actual_pos"],
            color=_COLOR_ACTUAL, linewidth=1.6, label="Actual",
            alpha=0.90, zorder=4, **act_style,
        )

        if res.get("displacement"):
            y_label = f"Δ position [{unit}]"
        else:
            y_label = f"Position [{unit}]"
        ax_traj.set_ylabel(y_label)
        ax_traj.set_title(f"{label}", fontweight="bold", pad=6)
        ax_traj.legend(
            loc="lower right", frameon=True, framealpha=0.9,
            edgecolor="#D1D5DB", fancybox=True,
        )
        ax_traj.grid(True, color=_COLOR_GRID, linewidth=0.6, zorder=0)
        ax_traj.tick_params(axis="both", length=3)

        # ----- Right: tracking error -----
        ax_err.fill_between(
            t, 0, res["error"],
            color=_COLOR_FILL, alpha=0.45, zorder=1,
        )
        ax_err.plot(
            t, res["error"],
            color=_COLOR_ERROR, linewidth=1.2, zorder=3,
        )
        ax_err.axhline(0, color="#9CA3AF", linewidth=0.8, linestyle="-", zorder=2)

        err_label = f"Error (ΔPlan − ΔAct) [{unit}]" if res.get("displacement") \
            else f"Error (Plan − Act) [{unit}]"
        ax_err.set_ylabel(err_label)
        ax_err.set_title(f"{label} — Error", fontweight="bold", pad=6)

        rmse_str = f"RMSE = {res['rmse']:.4f} {unit}"
        ax_err.text(
            0.97, 0.93, rmse_str,
            transform=ax_err.transAxes,
            ha="right", va="top",
            fontsize=10, fontweight="bold", color="#92400E",
            bbox=dict(
                boxstyle="round,pad=0.4",
                fc=_COLOR_RMSE_BG, ec=_COLOR_RMSE_BD,
                linewidth=1.2, alpha=0.95,
            ),
        )
        ax_err.grid(True, color=_COLOR_GRID, linewidth=0.6, zorder=0)
        ax_err.tick_params(axis="both", length=3)

    # Common x-labels on the bottom row only
    for col in range(2):
        axes[-1, col].set_xlabel("Time [s]", fontweight="medium")

    fig.align_ylabels(axes[:, 0])
    fig.align_ylabels(axes[:, 1])

    plt.savefig("hil_trajectory_eval.png", dpi=180, bbox_inches="tight",
                facecolor="white", edgecolor="none")
    print("\n[INFO] Figure saved to hil_trajectory_eval.png")
    plt.show()


# ---------------------------------------------------------------------------
# Velocity & Jerk Evaluation
# ---------------------------------------------------------------------------

# Velocity/jerk plot colours (complement the position palette)
_COLOR_VEL_PLAN   = "#2563EB"   # blue-600  (same as position plan)
_COLOR_VEL_ACTUAL = "#F97316"   # orange-500 (same as position actual)
_COLOR_VEL_ERROR  = "#DC2626"   # red-600
_COLOR_VEL_FILL   = "#FECACA"   # red-200
_COLOR_JERK_PLAN  = "#7C3AED"   # violet-700
_COLOR_JERK_ACT   = "#D97706"   # amber-600
_COLOR_JERK_FILL  = "#FDE68A"   # amber-200


def compute_velocity_profiles(
    results: dict,
    smooth_window: int = 11,
    smooth_polyorder: int = 3,
) -> dict:
    """Differentiate interpolated planned and actual position traces to obtain
    velocity and jerk profiles in the analysed overlap window.

    Both series are already trimmed and time-aligned inside *results* (the
    output of :func:`analyse`), so ``t`` is in seconds and positions are in
    metres or radians.

    Planned velocity
        ``np.gradient`` on the densely-sampled ``planned_interp`` array.
        (The interpolant is evaluated at every actual timestamp, so the
        numerical derivative is a faithful approximation of the analytic
        spline derivative.)

    Actual velocity
        ``np.gradient`` on the ``actual_pos`` array, followed by a
        Savitzky-Golay filter to suppress encoder quantisation noise before
        jerk is computed.  The raw (unsmoothed) actual velocity is also
        retained for reference.

    Jerk
        ``np.gradient`` applied to the (smoothed) actual velocity and to
        the planned velocity.  A second Savitzky-Golay pass is applied to
        the actual jerk so the trace is readable on the plot.

    Parameters
    ----------
    results : dict
        Output of :func:`analyse`.  Must contain at least ``t``,
        ``planned_interp``, and ``actual_pos`` per joint.
    smooth_window : int
        Savitzky-Golay window length (samples).  Must be odd; the function
        clamps and adjusts it automatically.  Increase for noisier signals.
    smooth_polyorder : int
        Polynomial order for Savitzky-Golay filter (default 3).

    Returns
    -------
    vel_results : dict[str, dict | None]
        Per joint (keys match :data:`JOINT_NAMES`):

        ``t``               – normalised time [s] (same as ``results[j]["t"]``)
        ``planned_vel``     – planned velocity [unit/s]
        ``actual_vel_raw``  – actual velocity before smoothing [unit/s]
        ``actual_vel``      – Savitzky-Golay smoothed actual velocity [unit/s]
        ``vel_error``       – velocity tracking error (planned − actual) [unit/s]
        ``vel_rmse``        – RMSE of velocity error [unit/s]
        ``planned_jerk``    – jerk derived from planned velocity [unit/s³]
        ``actual_jerk``     – jerk derived from smoothed actual velocity [unit/s³]
        ``max_actual_jerk`` – peak |jerk| of the actual trajectory [unit/s³]
        ``rms_actual_jerk`` – RMS jerk of the actual trajectory [unit/s³]
        ``max_actual_vel``  – peak |velocity| of the actual trajectory [unit/s]
    """
    vel_results: dict = {}

    for jname in JOINT_NAMES:
        res = results.get(jname)
        if res is None:
            vel_results[jname] = None
            continue

        t        = res["t"]
        pos_act  = res["actual_pos"]
        pos_plan = res["planned_interp"]

        if t.size < 5:
            vel_results[jname] = None
            continue

        # ---- Adaptive Savitzky-Golay window ----
        # Must be odd, >= polyorder+2, and < data length.
        win = smooth_window
        win = min(win, t.size - (1 if t.size % 2 == 0 else 0))
        win = max(win, smooth_polyorder + 2)
        if win % 2 == 0:
            win += 1
        can_smooth = t.size >= win

        # ---- Velocity (central differences) ----
        vel_plan    = np.gradient(pos_plan, t)
        vel_act_raw = np.gradient(pos_act,  t)

        if can_smooth:
            vel_act = savgol_filter(vel_act_raw, win, smooth_polyorder)
        else:
            vel_act = vel_act_raw.copy()

        # ---- Jerk = d(vel)/dt ----
        jerk_plan   = np.gradient(vel_plan, t)
        jerk_act_raw = np.gradient(vel_act, t)

        if can_smooth:
            jerk_act = savgol_filter(jerk_act_raw, win, smooth_polyorder)
        else:
            jerk_act = jerk_act_raw.copy()

        vel_error = vel_plan - vel_act
        vel_rmse  = float(np.sqrt(np.mean(vel_error ** 2)))

        vel_results[jname] = {
            "t":               t,
            "planned_vel":     vel_plan,
            "actual_vel_raw":  vel_act_raw,
            "actual_vel":      vel_act,
            "vel_error":       vel_error,
            "vel_rmse":        vel_rmse,
            "planned_jerk":    jerk_plan,
            "actual_jerk":     jerk_act,
            "max_actual_jerk": float(np.max(np.abs(jerk_act))),
            "rms_actual_jerk": float(np.sqrt(np.mean(jerk_act ** 2))),
            "max_actual_vel":  float(np.max(np.abs(vel_act))),
        }

    return vel_results


def print_velocity_summary(vel_results: dict) -> None:
    """Print a concise per-joint velocity & jerk report.

    For flexible-load crane systems, high jerk is the primary driver of
    payload sway.  The table includes:

    * Velocity RMSE      – how closely the controller tracks the planned speed
    * Peak actual |vel|  – peak speed reached on the joint
    * Max |jerk|         – instantaneous sway excitation potential
    * RMS jerk           – average jerk energy (lower is smoother)
    """
    print("\n" + "=" * 66)
    print("  Velocity & Jerk Summary (sway-risk indicators)")
    print("=" * 66)
    header = (
        f"  {'Joint':<18s}  {'Vel-RMSE':>10s}  "
        f"{'Peak|vel|':>10s}  {'Max|jerk|':>12s}  {'RMS-jerk':>10s}"
    )
    print(header)
    print("  " + "-" * 62)
    for jname in JOINT_NAMES:
        vr = vel_results.get(jname)
        _, unit = _JOINT_LABELS.get(jname, (jname, ""))
        u_v = f"{unit}/s"
        u_j = f"{unit}/s³"
        if vr is None:
            print(f"  {jname:<18s}  {'—':>10s}  {'—':>10s}  {'—':>12s}  {'—':>10s}")
        else:
            print(
                f"  {jname:<18s}  "
                f"{vr['vel_rmse']:>9.5f} {u_v}  "
                f"{vr['max_actual_vel']:>9.5f} {u_v}  "
                f"{vr['max_actual_jerk']:>11.5f} {u_j}  "
                f"{vr['rms_actual_jerk']:>9.5f} {u_j}"
            )
    print("=" * 66)
    print(
        "  Note: large Max|jerk| or RMS-jerk relative to motion scale\n"
        "  indicates risk of payload sway. Consider a smoother S-curve\n"
        "  or lower acceleration limit on the offending joint."
    )
    print("=" * 66)


def plot_velocity_results(
    vel_results: dict,
    smooth_window: int = 11,
    smooth_polyorder: int = 3,
) -> None:
    """Create a 3 × 3 subplot figure for velocity / jerk evaluation.

    Layout
    ------
    Column 0  Velocity profile  – planned (blue) vs smoothed actual (orange)
               overlaid with raw actual velocity (faint grey) so the
               smoothing extent is visible.
    Column 1  Velocity error    – (planned − smoothed actual), RMSE badge.
    Column 2  Jerk overlay      – planned jerk (violet) vs smoothed actual
               jerk (amber), filled area under actual jerk for sway-risk
               emphasis.

    The figure is saved to ``hil_velocity_eval.png``.
    """
    plt.rcParams.update({
        "font.family":      "sans-serif",
        "font.size":        10,
        "axes.titlesize":   11,
        "axes.labelsize":   10,
        "xtick.labelsize":  9,
        "ytick.labelsize":  9,
        "legend.fontsize":  9,
        "figure.facecolor": "white",
        "axes.facecolor":   "#FAFAFA",
        "axes.edgecolor":   "#D1D5DB",
        "axes.linewidth":   0.8,
        "xtick.color":      "#6B7280",
        "ytick.color":      "#6B7280",
        "axes.labelcolor":  "#374151",
        "axes.titlecolor":  "#111827",
    })

    fig, axes = plt.subplots(
        nrows=3, ncols=3,
        figsize=(18, 10),
        constrained_layout=True,
    )
    fig.suptitle(
        "HIL Velocity & Jerk Evaluation — 3-Axis Crane",
        fontsize=15, fontweight="bold", color="#111827", y=0.98,
    )

    col_titles = ["Velocity Profile", "Velocity Error", "Jerk"]
    for col, ctitle in enumerate(col_titles):
        axes[0, col].set_title(ctitle, fontweight="bold", pad=8, fontsize=12)

    no_data_kwargs = dict(ha="center", va="center", fontsize=11, color="#9CA3AF")

    for row, jname in enumerate(JOINT_NAMES):
        vr = vel_results.get(jname)
        label, unit = _JOINT_LABELS.get(jname, (jname, ""))
        u_v = f"{unit}/s"
        u_j = f"{unit}/s³"

        ax_vel  = axes[row, 0]
        ax_err  = axes[row, 1]
        ax_jerk = axes[row, 2]

        # Row label on the left velocity column
        ax_vel.set_ylabel(f"{label}\n[{u_v}]", fontweight="bold")

        if vr is None:
            for ax in (ax_vel, ax_err, ax_jerk):
                ax.text(0.5, 0.5, "No data", transform=ax.transAxes, **no_data_kwargs)
                ax.set_yticks([])
            continue

        t = vr["t"]

        # ---- Col 0: Velocity profile ----
        ax_vel.plot(
            t, vr["actual_vel_raw"],
            color="#9CA3AF", linewidth=0.8, alpha=0.45, label="Actual (raw)", zorder=2,
        )
        ax_vel.plot(
            t, vr["planned_vel"],
            color=_COLOR_VEL_PLAN, linewidth=2.0, label="Planned", zorder=4,
        )
        ax_vel.plot(
            t, vr["actual_vel"],
            color=_COLOR_VEL_ACTUAL, linewidth=1.6, alpha=0.9, label="Actual (smoothed)", zorder=3,
        )
        ax_vel.axhline(0, color="#9CA3AF", linewidth=0.6, linestyle="--", zorder=1)
        ax_vel.legend(loc="lower right", frameon=True, framealpha=0.9,
                      edgecolor="#D1D5DB", fancybox=True)
        ax_vel.grid(True, color=_COLOR_GRID, linewidth=0.6, zorder=0)
        ax_vel.tick_params(axis="both", length=3)

        # ---- Col 1: Velocity error ----
        ax_err.fill_between(
            t, 0, vr["vel_error"],
            color=_COLOR_VEL_FILL, alpha=0.45, zorder=1,
        )
        ax_err.plot(
            t, vr["vel_error"],
            color=_COLOR_VEL_ERROR, linewidth=1.2, zorder=3,
        )
        ax_err.axhline(0, color="#9CA3AF", linewidth=0.8, linestyle="-", zorder=2)
        ax_err.set_ylabel(f"Vel Error [{u_v}]")

        rmse_str = f"Vel RMSE = {vr['vel_rmse']:.4f} {u_v}"
        ax_err.text(
            0.97, 0.93, rmse_str,
            transform=ax_err.transAxes,
            ha="right", va="top",
            fontsize=10, fontweight="bold", color="#92400E",
            bbox=dict(
                boxstyle="round,pad=0.4",
                fc=_COLOR_RMSE_BG, ec=_COLOR_RMSE_BD,
                linewidth=1.2, alpha=0.95,
            ),
        )
        ax_err.grid(True, color=_COLOR_GRID, linewidth=0.6, zorder=0)
        ax_err.tick_params(axis="both", length=3)

        # ---- Col 2: Jerk overlay ----
        ax_jerk.fill_between(
            t, 0, vr["actual_jerk"],
            color=_COLOR_JERK_FILL, alpha=0.35, zorder=1,
        )
        ax_jerk.plot(
            t, vr["planned_jerk"],
            color=_COLOR_JERK_PLAN, linewidth=1.8, label="Planned jerk", zorder=3,
        )
        ax_jerk.plot(
            t, vr["actual_jerk"],
            color=_COLOR_JERK_ACT, linewidth=1.6, alpha=0.9,
            label="Actual jerk (smoothed)", zorder=4,
        )
        ax_jerk.axhline(0, color="#9CA3AF", linewidth=0.6, linestyle="--", zorder=2)
        ax_jerk.set_ylabel(f"Jerk [{u_j}]")

        jerk_str = (
            f"Max|j| = {vr['max_actual_jerk']:.4f} {u_j}\n"
            f"RMS j  = {vr['rms_actual_jerk']:.4f} {u_j}"
        )
        ax_jerk.text(
            0.97, 0.93, jerk_str,
            transform=ax_jerk.transAxes,
            ha="right", va="top",
            fontsize=9, fontweight="bold", color="#78350F",
            bbox=dict(
                boxstyle="round,pad=0.4",
                fc="#FEF3C7", ec="#D97706",
                linewidth=1.2, alpha=0.95,
            ),
        )
        ax_jerk.legend(loc="lower right", frameon=True, framealpha=0.9,
                       edgecolor="#D1D5DB", fancybox=True)
        ax_jerk.grid(True, color=_COLOR_GRID, linewidth=0.6, zorder=0)
        ax_jerk.tick_params(axis="both", length=3)

    # Common x-axis labels on bottom row
    for col in range(3):
        axes[-1, col].set_xlabel("Time [s]", fontweight="medium")

    fig.align_ylabels(axes[:, 0])
    fig.align_ylabels(axes[:, 1])
    fig.align_ylabels(axes[:, 2])

    plt.savefig("hil_velocity_eval.png", dpi=180, bbox_inches="tight",
                facecolor="white", edgecolor="none")
    print("[INFO] Velocity figure saved to hil_velocity_eval.png")
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="HIL Trajectory Fidelity Evaluator for 3-Axis Crane"
    )
    parser.add_argument(
        "bag", type=str,
        help="Path to the ROS 2 bag directory (mcap or sqlite3)"
    )
    parser.add_argument(
        "--plan", nargs="+", metavar="POS@TIME",
        help=(
            "Planned trajectory waypoints: hook[m],trolley[m],slewing[rad]@time_sec "
            "(unless --degrees or --deg-joints). "
            "Example: --plan 0,0,0@0 0.5,0.8,1.57@7"
        ),
    )
    parser.add_argument(
        "--degrees", action="store_true", default=False,
        help=(
            "Convert ALL --plan positions from degrees to radians. "
            "For mixed joint types use --deg-joints instead."
        ),
    )
    parser.add_argument(
        "--deg-joints", nargs="+", metavar="JOINT",
        help=(
            "Convert only the listed joints from degrees to radians. "
            "Example: --deg-joints slewing_joint"
        ),
    )
    parser.add_argument(
        "--interp", choices=["linear", "cubic"], default="cubic",
        help=(
            "Interpolation method for the planned trajectory. "
            "'cubic' (default) uses a cubic Hermite spline with zero "
            "velocity at waypoints, matching JointTrajectoryController. "
            "'linear' uses piecewise-linear interpolation."
        ),
    )
    parser.add_argument(
        "--align-plan-to",
        choices=["bag_start", "first_motion", "first_motion_global"],
        default="first_motion",
        help=(
            "Where to place the --plan t=0 waypoint in bag time. "
            "'bag_start': first joint_states timestamp. "
            "'first_motion' (default): **per-joint** first motion — each axis "
            "gets its own anchor (use when hook/slewing/trolley start at different times). "
            "'first_motion_global': one anchor = earliest motion among all joints "
            "(legacy; can leave some axes flat in the 7.2 s window)."
        ),
    )
    parser.add_argument(
        "--motion-threshold",
        type=float,
        default=0.02,
        metavar="FLOAT",
        help=(
            "With --align-plan-to first_motion: minimum position change (m or rad) "
            "from each joint's first sample to count as motion start."
        ),
    )
    parser.add_argument(
        "--plan-offset-sec",
        type=float,
        default=0.0,
        help=(
            "Seconds added after the plan anchor (from --align-plan-to). "
            "Use to fine-tune alignment with the recorded trajectory."
        ),
    )
    parser.add_argument(
        "--best-fit",
        action="store_true",
        default=False,
        help=(
            "Search a small alignment grid and pick the lowest-RMSE fit that "
            "still contains real motion. Only applies with --plan."
        ),
    )
    parser.add_argument(
        "--best-fit-min-motion",
        type=float,
        default=0.05,
        metavar="FLOAT",
        help=(
            "Minimum motion span required in every joint during --best-fit. "
            "Prevents picking a trivially low-RMSE window where nothing moved."
        ),
    )
    parser.add_argument(
        "--auto-duration",
        action="store_true",
        default=False,
        help=(
            "Scale each joint's plan time independently so that the plan "
            "duration matches the actual motion duration detected in the bag. "
            "Useful when each axis moves at a different speed on real hardware "
            "and a single @time in the CLI cannot fit all joints."
        ),
    )
    parser.add_argument(
        "--plan-motor-deg",
        action="store_true",
        default=False,
        help=(
            "Interpret --plan waypoints as CAN motor degrees (same as "
            "/hoist/crane_position etc.), converted to joint_states units "
            "using the same formulas as moveit_bridge.cpp (hook/trolley→m, "
            "slewing→rad)."
        ),
    )
    parser.add_argument(
        "--invert-prismatic",
        action="store_true",
        default=False,
        help=(
            "Negate hook_joint and trolley_joint positions from the bag before "
            "analysis if motor/URDF positive direction is opposite to MoveIt."
        ),
    )
    parser.add_argument(
        "--invert-slewing",
        action="store_true",
        default=False,
        help="Negate slewing_joint actual positions before analysis (rare axis sign mismatch).",
    )
    parser.add_argument(
        "--no-auto-invert",
        action="store_true",
        default=False,
        help=(
            "Disable automatic sign correction: if Pearson r (Δ plan vs Δ act) is "
            "strongly negative for a joint, the script would negate that joint's "
            "actual series and re-run analysis once."
        ),
    )
    parser.add_argument(
        "--no-actual-step-plot",
        action="store_true",
        default=False,
        help="Draw actual as linear interpolation between samples (default: steps-post).",
    )
    parser.add_argument(
        "--absolute-positions",
        action="store_true",
        default=False,
        help=(
            "Compare absolute joint positions (no subtraction at overlap start). "
            "Default is displacement mode: zero both series at the first overlap "
            "sample so the plot shows tracking of motion."
        ),
    )
    parser.add_argument(
        "--actual-topic", type=str, default=TOPIC_ACTUAL_DEFAULT,
        help=(
            "JointState topic to use as measured/actual trajectory. "
            "Examples: /joint_states, /joint_states_merged, /joint_states_dynamics"
        ),
    )
    parser.add_argument(
        "--velocity-eval",
        action="store_true",
        default=False,
        help=(
            "Compute and plot velocity & jerk profiles after position analysis. "
            "Produces an additional figure (hil_velocity_eval.png) with "
            "velocity overlay, velocity tracking error, and jerk — "
            "key sway-risk indicators for flexible-load crane systems."
        ),
    )
    parser.add_argument(
        "--vel-smooth-window",
        type=int,
        default=11,
        metavar="N",
        help=(
            "Savitzky-Golay filter window (samples, odd) applied to the actual "
            "velocity before jerk is computed. Increase for noisier encoder signals. "
            "(default: 11)"
        ),
    )
    parser.add_argument(
        "--vel-smooth-polyorder",
        type=int,
        default=3,
        metavar="K",
        help=(
            "Polynomial order for the Savitzky-Golay velocity smoother. "
            "Must be less than --vel-smooth-window. (default: 3)"
        ),
    )
    # Use parse_known_args so that negative-number plan waypoints
    # (e.g. -0.22,-0.35,30@7) are not rejected as unknown flags.
    args, unknown = parser.parse_known_args()
    plan_extras = [u for u in unknown if "@" in u]
    non_plan = [u for u in unknown if "@" not in u]
    if non_plan:
        parser.error(f"unrecognized arguments: {' '.join(non_plan)}")
    if plan_extras:
        if args.plan is None:
            args.plan = plan_extras
        else:
            args.plan.extend(plan_extras)

    bag_path = Path(args.bag)
    if not bag_path.exists():
        print(f"[ERROR] Bag path does not exist: {bag_path}")
        sys.exit(1)

    # --- Step 1: Extract actual data from bag ---
    print(f"[INFO] Reading bag: {bag_path}")
    print(f"[INFO] Using actual topic: {args.actual_topic}")
    base_actual, actual_info = extract_actual(str(bag_path), actual_topic=args.actual_topic)
    if actual_info["topic_msg_count"] == 0:
        # Missing/empty topic: [ERROR] already printed inside extract_actual when absent
        sys.exit(1)
    if all(base_actual[j][0].size == 0 for j in JOINT_NAMES):
        sample = actual_info.get("sample_joint_names") or []
        print(
            f"\n[ERROR] Topic '{args.actual_topic}' has {actual_info['topic_msg_count']} message(s), "
            f"but none of {JOINT_NAMES} appear in JointState.name."
        )
        if sample:
            print(f"        Sample names in message: {sample[:20]}{'...' if len(sample) > 20 else ''}")
        sys.exit(1)

    invert_set: set[str] = set()
    if args.invert_prismatic:
        invert_set |= {"hook_joint", "trolley_joint"}
        print("[INFO] --invert-prismatic: negating hook_joint and trolley_joint actual positions.")
    if args.invert_slewing:
        invert_set.add("slewing_joint")
        print("[INFO] --invert-slewing: negating slewing_joint actual positions.")

    actual = apply_invert_joints(base_actual, invert_set)
    print_actual_summary(actual)

    # --- Step 2: Get planned trajectory ---
    if args.plan:
        if args.best_fit:
            if args.invert_prismatic or args.invert_slewing:
                print("[WARN] --best-fit ignores manual invert flags and searches its own best sign correction.")
            planned, actual, results, invert_set, best_fit = find_best_fit_configuration(base_actual, args)
            print_actual_summary(actual)
        else:
            best_fit = None
        print(f"[INFO] Using planned trajectory from --plan ({len(args.plan)} waypoints)")
        if args.plan_motor_deg:
            print(
                "[INFO] --plan-motor-deg: converting waypoints with moveit_bridge "
                f"ratios (M_PER_DEG hook={METERS_PER_DEGREE_HOIST:.6g}, "
                f"trolley={METERS_PER_DEGREE_TROLLEY:.6g})."
            )
        if args.plan_motor_deg and (args.degrees or args.deg_joints):
            print(
                "[WARN] --plan-motor-deg ignores --degrees / --deg-joints "
                "(waypoints are motor CAN degrees for all three joints)."
            )
        if args.deg_joints and not args.plan_motor_deg:
            print(f"[INFO] Converting deg→rad for joints: {args.deg_joints}")
        elif args.degrees and not args.plan_motor_deg:
            print("[INFO] Converting ALL plan waypoints from degrees to radians.")
        if not args.best_fit:
            print(f"[INFO] Plan interpolation: {args.interp}")
            print(
                f"[INFO] Plan time anchor: --align-plan-to {args.align_plan_to}"
                + (
                    f", --motion-threshold {args.motion_threshold}"
                    if args.align_plan_to in ("first_motion", "first_motion_global")
                    else ""
                )
                + (f", --plan-offset-sec {args.plan_offset_sec}" if args.plan_offset_sec != 0.0 else "")
            )
            t_anchor = compute_plan_time_anchors(
                base_actual,
                align=args.align_plan_to,
                motion_threshold=args.motion_threshold,
                plan_offset_sec=args.plan_offset_sec,
            )
            planned = build_planned_from_cli(
                args.plan, base_actual,
                degrees=args.degrees,
                deg_joints=args.deg_joints,
                t_anchor=t_anchor,
                plan_in_motor_deg=args.plan_motor_deg,
            )
            if args.auto_duration:
                planned = auto_scale_plan_duration(
                    planned, actual,
                    motion_threshold=args.motion_threshold,
                )
                planned = auto_align_time_offset(
                    planned, actual,
                    interp_kind=args.interp,
                    displacement=not args.absolute_positions,
                )
    else:
        if args.best_fit:
            print("[ERROR] --best-fit requires --plan.")
            sys.exit(1)
        best_fit = None
        print("[INFO] Looking for planned trajectory in bag topics...")
        planned = extract_planned_from_bag(str(bag_path), base_actual)
        if planned is None:
            print("[ERROR] No planned trajectory found in bag.")
            print("        Use --plan to specify waypoints manually, e.g.:")
            print("          --plan 0,0,0@0 0.5,0.8,1.57@7")
            sys.exit(1)

    for jname in JOINT_NAMES:
        n_plan = planned[jname][0].size
        n_act  = base_actual[jname][0].size
        print(f"  {jname:20s}  planned pts: {n_plan:>6d}   actual pts: {n_act:>6d}")

    # --- Step 3: Analyse ---
    if args.best_fit:
        if best_fit["interp"] != args.interp:
            print(f"[INFO] --best-fit overrode --interp -> {best_fit['interp']}")
        if best_fit["displacement"] != (not args.absolute_positions):
            mode_lab = "displacement" if best_fit["displacement"] else "absolute"
            print(f"[INFO] --best-fit overrode comparison mode -> {mode_lab}")
    else:
        results = analyse(
            planned,
            actual,
            interp_kind=args.interp,
            displacement=not args.absolute_positions,
        )
        if not args.no_auto_invert:
            extra = auto_detect_inverted_joints(results) - invert_set
            if extra:
                print(
                    f"[INFO] Auto-invert {sorted(extra)} (strong negative correlation "
                    "between Δ plan and Δ act). Re-running. Use --no-auto-invert to disable."
                )
                invert_set |= extra
                actual = apply_invert_joints(base_actual, invert_set)
                results = analyse(
                    planned,
                    actual,
                    interp_kind=args.interp,
                    displacement=not args.absolute_positions,
                )
    print_correlation_summary(results)
    warn_scale_and_units(planned, results)

    # --- Step 4: Report RMSE ---
    print("\n" + "=" * 50)
    print("  RMSE Results (per joint)")
    print("=" * 50)
    for jname in JOINT_NAMES:
        res = results.get(jname)
        if res is None:
            print(f"  {jname:20s}  —  (no overlapping data)")
        else:
            print(f"  {jname:20s}  RMSE = {res['rmse']:.6f}")
    print("=" * 50)

    # --- Step 5: Visualise ---
    plot_results(results, step_actual=not args.no_actual_step_plot)

    # --- Step 6: Velocity & Jerk evaluation (optional) ---
    if args.velocity_eval:
        print("\n[INFO] Running velocity & jerk evaluation...")
        vel_results = compute_velocity_profiles(
            results,
            smooth_window=args.vel_smooth_window,
            smooth_polyorder=args.vel_smooth_polyorder,
        )
        print_velocity_summary(vel_results)
        plot_velocity_results(
            vel_results,
            smooth_window=args.vel_smooth_window,
            smooth_polyorder=args.vel_smooth_polyorder,
        )


if __name__ == "__main__":
    main()
