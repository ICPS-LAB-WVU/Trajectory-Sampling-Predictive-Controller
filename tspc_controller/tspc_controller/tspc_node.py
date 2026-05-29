#!/usr/bin/env python3
import math
import numpy as np
from dataclasses import dataclass
import cvxpy  # not used anymore but kept for compatibility with your env
from scipy.linalg import block_diag  # also unused, safe to remove later
from scipy.spatial import transform
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class MPCSettings:
    """ Dataclass to store MPC settings and hyperparameters """
    node: Node

    def __post_init__(self):

        # Dimensions
        self.state_size = 4  # x, y, v, yaw
        self.input_size = 2  # accel, steering
        # Desired cruising speed (used by candidate trajectories)
        self.candidate_speed = self.node.get_parameter("candidate_speed").value

        # R Matrix: Input Cost (kept for future tuning if needed)
        self.input_cost = np.diag([
            self.node.get_parameter("input_cost_accel").value,
            self.node.get_parameter("input_cost_steering").value
        ])

        # Rd Matrix: Input Rate Cost
        self.input_rate_cost = np.diag([
            self.node.get_parameter("input_rate_cost_accel").value,
            self.node.get_parameter("input_rate_cost_steering").value
        ])

        # Q Matrix: State Error Cost
        self.state_cost = np.diag([
            self.node.get_parameter("state_cost_x").value,
            self.node.get_parameter("state_cost_y").value,
            self.node.get_parameter("state_cost_v").value,
            self.node.get_parameter("state_cost_yaw").value
        ])

        # Qf Matrix: Final State Error Cost
        self.final_state_cost = np.diag([
            self.node.get_parameter("final_state_cost_x").value,
            self.node.get_parameter("final_state_cost_y").value,
            self.node.get_parameter("final_state_cost_v").value,
            self.node.get_parameter("final_state_cost_yaw").value
        ])

        # MPC Parameters
        self.horizon = self.node.get_parameter("horizon").value
        self.search_idx = self.node.get_parameter("search_idx").value
        self.dt = self.node.get_parameter("dt").value
        self.step_length = self.node.get_parameter("step_length").value

        # Vehicle Parameters
        self.veh_length = self.node.get_parameter("veh_length").value
        self.veh_width = self.node.get_parameter("veh_width").value
        self.wheelbase = self.node.get_parameter("wheelbase").value
        self.min_steer = self.node.get_parameter("min_steer").value
        self.max_steer = self.node.get_parameter("max_steer").value
        self.max_steer_rate = self.node.get_parameter("max_steer_rate").value
        self.max_speed = self.node.get_parameter("max_speed").value
        self.min_speed = self.node.get_parameter("min_speed").value
        self.max_accel = self.node.get_parameter("max_accel").value


@dataclass
class VehicleState:
    x: float = 0.0   # x position [m]
    y: float = 0.0   # y position [m]
    delta: float = 0.0  # steering angle [rad]
    v: float = 0.0   # velocity [m/s]
    yaw: float = 0.0  # heading angle [rad]
    yaw_rate: float = 0.0  # yaw rate [rad/s]
    beta: float = 0.0  # slip angle [rad]


class MPCNode(Node):
    """
    A Node implementing a raceline-following, LiDAR-reactive MPC
    with a fan of candidate trajectories.
    """

    def __init__(self):
        super().__init__('mpc_node')

        # Initialize MPC settings and variables
        self.setup_parameters()
        self.settings = MPCSettings(self)
        self.prev_steer = 0.0
        self.prev_accel = 0.0
        self.last_scan = None
        self.use_real = self.get_parameter("real_environment").value
        self.map_filename = self.get_parameter("csv_file").value

        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.rot_matrix = np.identity(3)

        # Ego pose at current MPC step (global)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.raceline_steer = 0.0

        # Hyperparameters for reactive obstacle avoidance - Gap analysis
        self.downsample_gap = self.get_parameter("downsampling").value  # Downsampling factor for lidar data
        self.max_sight = self.get_parameter("max_sight").value         # Maximum range of lidar sensor
        self.gap_threshold = self.get_parameter("gap_threshold").value  # Minimum gap size for obstacle avoidance

        # Topics
        pose_topic = "/pf/viz/inferred_pose" if self.use_real else "/ego_racecar/odom"
        odom_topic = "/pf/viz/inferred_pose" if self.use_real else "/ego_racecar/odom"
        drive_topic = "/drive"
        vis_ref_topic = "/ref_traj_marker"
        vis_wpts_topic = "/waypoints_marker"
        vis_pred_topic = "/pred_path_marker"

        # Initialize variables for reactive obstacle avoidance
        self.processed_ranges = None  # Processed lidar data
        self.current_speed = 0.0      # Current vehicle speed

        self.gap_steer = 0.0          # desired steering from gap center
        self.gap_speed_desired = self.settings.candidate_speed

        # flag: do we have a blocking obstacle in front?
        self.obstacle_ahead = False
        self.front_clearance = self.max_sight


        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped if self.use_real else Odometry,
            pose_topic,
            self.pose_update,
            1
        )
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(
            PoseStamped if self.use_real else Odometry,
            odom_topic,
            self.odometry_update,
            1
        )

        # Publishers for drive commands and visualization markers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.drive_msg = AckermannDriveStamped()
        self.ref_traj_pub = self.create_publisher(Marker, vis_ref_topic, 1)
        self.ref_traj_marker = Marker()
        self.wpts_pub = self.create_publisher(Marker, vis_wpts_topic, 1)
        self.wpts_marker = Marker()
        self.pred_path_pub = self.create_publisher(MarkerArray, vis_pred_topic, 1)

        # Load waypoints from CSV file
        map_dir = os.path.abspath(os.path.join('src/jour_ws/src', 'csv_data'))
        self.waypoints = np.loadtxt(
            os.path.join(map_dir, self.map_filename + '.csv'),
            delimiter=';',
            skiprows=0
        )
        # After loading self.waypoints
        self.speed_scale = self.get_parameter("speed_scale").value  # add this param below
        self.speed_profile = self.waypoints[:, 5] * self.speed_scale


        # Apply the same yaw fix used in mpc.py for Levine
        if self.map_filename == 'Catalunya_fast':
            # Heading is in column 3, same as in mpc.py
            self.waypoints[:, 3] += math.pi / 2.0

        # Draw the waypoints once
        self.show_waypoints()


        self.get_logger().info("Fan-of-trajectories MPC node initialized.")

    # -----------------------------------------------------------
    # Callback Functions
    # -----------------------------------------------------------

    def lidar_callback(self, scan_msg):
        """ Process incoming LaserScan data. """
        self.last_scan = scan_msg
        # Only keep a sector for gap processing (e.g., roughly forward)
        ranges = np.array(scan_msg.ranges[180:899])
        self.processed_ranges = self.preprocess_laser_data(ranges)

    def odometry_update(self, msg):
        """ Update the current vehicle speed from odometry. """
        if not self.use_real:
            lin = msg.twist.twist.linear
            self.current_speed = math.sqrt(lin.x**2 + lin.y**2)
        self.get_logger().info(f"Current speed: {self.current_speed:.2f} m/s")

    def pose_update(self, pose_msg):
        """
        Process pose message to update state, do gap analysis,
        and run fan-of-trajectories predictive control.
        """
        self.update_rotation_matrix(pose_msg)
        vehicle_state = self.get_vehicle_state(pose_msg)

        # Store current pose for transforms (global)
        self.current_x = vehicle_state.x
        self.current_y = vehicle_state.y
        self.current_yaw = vehicle_state.yaw

        # --- Gap-based analysis for steering bias & speed ---
        if self.processed_ranges is not None:
            gap_start, gap_end = self.get_max_gap(self.processed_ranges)
            self.get_logger().info(
                f"Max gap: start={gap_start}, end={gap_end} (length={gap_end - gap_start})"
            )

            if self.last_scan is not None:
                self.gap_steer, self.gap_speed_desired = self.compute_gap_steer_and_speed(
                    self.last_scan, gap_start, gap_end
                )
                self.get_logger().info(
                    f"Gap steer target={self.gap_steer:.3f} rad, "
                    f"gap speed target={self.gap_speed_desired:.2f} m/s"
                )

        # --- Reference trajectory from raceline (for cost only) ---
        ref_traj = self.compute_reference_traj(
            vehicle_state,
            self.waypoints[:, 1],
            self.waypoints[:, 2],
            self.waypoints[:, 3],
            self.speed_profile
        )
        self.show_reference_traj(ref_traj)
        v_csv = float(ref_traj[2, 0])


        # --- Fan-of-trajectories predictive control ---
        steer_cmd, speed_cmd, global_paths, best_idx = self.run_candidate_mpc(
            vehicle_state, ref_traj
        )

        # Clip by gap-based desired speed
        speed_cmd = min(speed_cmd, self.gap_speed_desired)

        self.drive_msg.drive.steering_angle = steer_cmd
        self.drive_msg.drive.speed = (-1.0 if self.use_real else 1.0) * speed_cmd
        self.get_logger().info(
            f"v_csv={v_csv:.2f}  v_odom={self.current_speed:.2f}  v_gap={self.gap_speed_desired:.2f}  v_cmd={speed_cmd:.2f}"
        )

        self.drive_pub.publish(self.drive_msg)
        self.get_logger().info(f"Command: steering={steer_cmd:.3f}, speed={speed_cmd:.3f}")

        # Visualize candidate fan
        self.show_predicted_paths(global_paths, best_idx)
        self.show_waypoints()

    # -----------------------------------------------------------
    # Vehicle State and Candidate-Trajectory MPC Functions
    # -----------------------------------------------------------

    def update_rotation_matrix(self, pose_msg):
        """ Update the rotation matrix from the current pose. """
        orien = pose_msg.pose.orientation if self.use_real else pose_msg.pose.pose.orientation
        quat = [orien.x, orien.y, orien.z, orien.w]
        self.rot_matrix = transform.Rotation.from_quat(quat).as_matrix()

    def get_vehicle_state(self, pose_msg):
        """ Extract vehicle state from pose message. """
        state = VehicleState()
        state.x = pose_msg.pose.position.x if self.use_real else pose_msg.pose.pose.position.x
        state.y = pose_msg.pose.position.y if self.use_real else pose_msg.pose.pose.position.y
        state.v = self.current_speed
        orient = pose_msg.pose.orientation if self.use_real else pose_msg.pose.pose.orientation
        q = [orient.x, orient.y, orient.z, orient.w]
        state.yaw = math.atan2(2 * (q[3]*q[2] + q[0]*q[1]),
                               1 - 2 * (q[1]**2 + q[2]**2))
        return state

    def get_nearest_point(self, pt, traj):
        """ Return the nearest point on a piecewise-linear trajectory. """
        diffs = traj[1:, :] - traj[:-1, :]
        l2s = diffs[:, 0]**2 + diffs[:, 1]**2
        dots = np.empty((traj.shape[0] - 1,))
        for i in range(dots.shape[0]):
            dots[i] = np.dot((pt - traj[i, :]), diffs[i, :])
        t_vals = dots / l2s
        t_vals[t_vals < 0.0] = 0.0
        t_vals[t_vals > 1.0] = 1.0
        projections = traj[:-1, :] + (t_vals * diffs.T).T
        dists = np.empty((projections.shape[0],))
        for i in range(dists.shape[0]):
            diff_vec = pt - projections[i]
            dists[i] = np.sqrt(np.sum(diff_vec * diff_vec))
        min_idx = np.argmin(dists)
        return projections[min_idx], dists[min_idx], t_vals[min_idx], min_idx

    def compute_reference_traj(self, state, cx, cy, cyaw, sp):
        """
        Calculate a reference trajectory based on the current vehicle state.

        This is now the same logic as calc_ref_trajectory() in mpc.py:
        - find nearest waypoint
        - move forward along the raceline using a step based on speed and step_length
        """
        ref_traj = np.zeros(
            (self.settings.state_size, self.settings.horizon + 1)
        )
        n_course = len(cx)

        # Nearest point on the global waypoints
        _, _, _, idx = self.get_nearest_point(
            np.array([state.x, state.y]),
            np.array([cx, cy]).T
        )

        # Initial reference state
        ref_traj[0, 0] = cx[idx]
        ref_traj[1, 0] = cy[idx]
        ref_traj[2, 0] = sp[idx]
        ref_traj[3, 0] = cyaw[idx]

        # Distance travelled per MPC step, based on current speed
        travel_dist = abs(state.v) * self.settings.dt
        d_index = travel_dist / self.settings.step_length

        # Do not let the index increment be < 1, otherwise path barely advances
        if d_index < 1.0:
            d_index = 1.0

        idx_list = int(idx) + np.insert(
            np.cumsum(np.repeat(d_index, self.settings.horizon)),
            0,
            0
        ).astype(int)
        idx_list[idx_list >= n_course] -= n_course

        ref_traj[0, :] = cx[idx_list]
        ref_traj[1, :] = cy[idx_list]
        ref_traj[2, :] = sp[idx_list]

        # Yaw unwrapping to avoid 2π jumps (same idea as mpc.py)
        angle_thresh = 4.5
        for i in range(len(cyaw)):
            if cyaw[i] - state.yaw > angle_thresh:
                cyaw[i] -= 2 * np.pi
            if state.yaw - cyaw[i] > angle_thresh:
                cyaw[i] += 2 * np.pi

        ref_traj[3, :] = cyaw[idx_list]
        return ref_traj


    # ---- local bicycle model for candidate rollout ----
    def propagate_local(self, x, y, yaw, v, a, delta):
        """Propagate a simple bicycle model in the local (ego) frame."""
        dt = self.settings.dt
        v_next = np.clip(v + a * dt, self.settings.min_speed, self.settings.max_speed)
        x_next = x + v_next * math.cos(yaw) * dt
        y_next = y + v_next * math.sin(yaw) * dt
        yaw_next = yaw + v_next / self.settings.wheelbase * math.tan(delta) * dt
        return x_next, y_next, yaw_next, v_next

    def generate_candidate_controls(self, v_target):
        """
        Generate a set of constant steering candidates.

        - If track is clear (no obstacle_ahead), the fan is narrow and
          centered around the raceline steering.
        - If there is an obstacle_ahead, we bias the fan around the gap steer
          and make it wider (overtaking / avoidance mode).
        """
        N = self.settings.horizon

        if not self.obstacle_ahead:
            # Raceline-tracking mode
            num_candidates = 9
            base = self.raceline_steer          # <--- raceline, not 0.0
            fan_half_angle = np.deg2rad(8.0)    # narrow fan around raceline
        else:
            # Obstacle-avoidance mode
            num_candidates = 17
            base = self.gap_steer
            fan_half_angle = np.deg2rad(30.0)   # wider fan to search more

        # Clamp base within steering limits
        base = np.clip(base, self.settings.min_steer, self.settings.max_steer)

        left  = np.clip(base + fan_half_angle, self.settings.min_steer, self.settings.max_steer)
        right = np.clip(base - fan_half_angle, self.settings.min_steer, self.settings.max_steer)
        if left < right:
            left, right = right, left

        steering_candidates = np.linspace(right, left, num_candidates)

        # Simple accel profile: try to go towards candidate_speed
        accel_candidates = []
        for _ in steering_candidates:
            accel_seq = np.zeros(N)
            # simple P controller on speed -> accel
            k_v = 2.0
            a = k_v * (v_target - self.current_speed)
            a = float(np.clip(a, -self.settings.max_accel, self.settings.max_accel))
            accel_seq[:] = a
            accel_candidates.append(accel_seq)

        return steering_candidates, accel_candidates




    def simulate_candidate(self, steer_const, accel_seq):
        """
        Simulate a single candidate in the local frame.
        Returns arrays x,y,yaw,v of length (N+1).
        """
        N = self.settings.horizon
        x = np.zeros(N + 1)
        y = np.zeros(N + 1)
        yaw = np.zeros(N + 1)
        v = np.zeros(N + 1)

        # start at origin in local frame, heading along +x
        # give it at least some forward speed so it accelerates quickly
        v[0] = max(self.current_speed, 1.5)

        for t in range(N):
            x[t+1], y[t+1], yaw[t+1], v[t+1] = self.propagate_local(
                x[t], y[t], yaw[t], v[t], accel_seq[t], steer_const
            )
        return x, y, yaw, v

    def check_collision_and_cost(self, x_local, y_local, steer_const, ref_traj):
        """
        Strong collision check + *horizon-wide* tracking cost.

        - Collision is checked in the local frame (same as before).
        - Tracking error is the sum of squared distances between the
          GLOBAL candidate points and the corresponding reference
          points at each time step t.
        - We switch weights depending on whether an obstacle is ahead:
            * obstacle_ahead = False  -> strongly hug raceline
            * obstacle_ahead = True   -> still track, but care a lot
                                        about clearance.
        """
        ranges = None
        angle_min = 0.0
        angle_inc = 0.0
        if self.last_scan is not None:
            ranges = np.array(self.last_scan.ranges)
            angle_min = self.last_scan.angle_min
            angle_inc = self.last_scan.angle_increment

        # --- transform whole candidate to GLOBAL frame once ---
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        x_global = self.current_x + cos_y * x_local - sin_y * y_local
        y_global = self.current_y + sin_y * x_local + cos_y * y_local

        # --- collision parameters ---
        safety_radius = 0.55      # "radius" of car + margin
        lidar_inflation = 0.30    # shrink free space this much
        min_clearance = self.max_sight
        collision = False
        substeps = 5              # sub-sampling along each segment

        # ---- collision check in LOCAL frame (same as before) ----
        for t in range(len(x_local) - 1):
            x0, y0 = x_local[t],     y_local[t]
            x1, y1 = x_local[t + 1], y_local[t + 1]

            for s in np.linspace(0.0, 1.0, substeps, endpoint=False):
                px = x0 + s * (x1 - x0)
                py = y0 + s * (y1 - y0)

                r = math.hypot(px, py)
                if r < 0.10:  # ignore very close to origin
                    continue

                theta = math.atan2(py, px)

                if ranges is not None:
                    idx = int((theta - angle_min) / angle_inc)

                    if 0 <= idx < len(ranges):
                        meas = ranges[idx]
                        if not math.isfinite(meas) or meas <= 0.0:
                            meas = self.max_sight
                    else:
                        meas = self.max_sight

                    # be conservative: obstacles are closer than they look
                    meas = max(meas - lidar_inflation, 0.0)

                    clearance = meas - r
                    min_clearance = min(min_clearance, clearance)

                    if clearance < safety_radius:
                        collision = True
                        break

            if collision:
                break

        # ---- horizon-wide tracking error (GLOBAL frame) ----
        # ref_traj has shape (4, horizon+1)
        N_ref = ref_traj.shape[1]
        T = min(len(x_local), N_ref)

        track_err = 0.0
        for t in range(T):
            dx = x_global[t] - ref_traj[0, t]
            dy = y_global[t] - ref_traj[1, t]
            track_err += dx * dx + dy * dy

        # forward progress in local frame (x direction)
        forward_progress = x_local[-1]

        # --- cost weights ---
        if self.obstacle_ahead:
            # avoidance / overtaking mode: still track, but clearance is key
            w_track = 5.0
            w_steer = 0.2
            w_prog  = 1.0
            w_clear = 8.0
        else:
            # free-track mode: really hug the raceline and move forward
            w_track = 10.0    # strong tracking
            w_steer = 0.1
            w_prog  = 2.0
            w_clear = 0.0     # only to discourage getting near walls
                              # (collision penalty still dominates)
        cost = (
            w_track * track_err
            + w_steer * (steer_const ** 2)
            - w_prog  * forward_progress
            - w_clear * max(min_clearance, 0.0)
        )

        if collision:
            cost += 1e6  # huge penalty

        return collision, cost




    def run_candidate_mpc(self, vehicle_state, ref_traj):
        v_target = float(np.clip(ref_traj[2, 0], self.settings.min_speed, self.settings.max_speed))

        steering_candidates, accel_candidates = self.generate_candidate_controls(v_target)
        global_paths = []
        best_cost = float('inf')
        best_idx = 0

        for idx, (steer_const, accel_seq) in enumerate(zip(steering_candidates, accel_candidates)):
            x_l, y_l, yaw_l, v_l = self.simulate_candidate(steer_const, accel_seq)
            collision, cost = self.check_collision_and_cost(x_l, y_l, steer_const, ref_traj)

            # transform local path to global for visualization
            cos_y = math.cos(self.current_yaw)
            sin_y = math.sin(self.current_yaw)
            x_g = self.current_x + cos_y * x_l - sin_y * y_l
            y_g = self.current_y + sin_y * x_l + cos_y * y_l

            path_global = np.zeros((4, self.settings.horizon + 1))
            path_global[0, :] = x_g
            path_global[1, :] = y_g
            path_global[2, :] = v_l
            path_global[3, :] = yaw_l + self.current_yaw
            global_paths.append(path_global)

            if cost < best_cost:
                best_cost = cost
                best_idx = idx

        best_steer = steering_candidates[best_idx]

        # speed = first-step rollout speed, but never exceed v_target (CSV profile)
        #_, _, _, v_l = self.simulate_candidate(best_steer, accel_candidates[best_idx])
        #best_speed = float(np.clip(v_l[1], self.settings.min_speed, self.settings.max_speed))
        #best_speed = min(best_speed, v_target)
        best_speed = float(np.clip(v_target, self.settings.min_speed, self.settings.max_speed))


        return best_steer, best_speed, global_paths, best_idx


    # -----------------------------------------------------------
    # Reactive Obstacle Detection Functions (gap follow style)
    # -----------------------------------------------------------
    def preprocess_laser_data(self, ranges):
        num_bins = int(len(ranges) / self.downsample_gap)
        proc = np.zeros(num_bins)
        for i in range(num_bins):
            window = ranges[i * self.downsample_gap:
                            (i + 1) * self.downsample_gap]
            proc[i] = sum(window) / self.downsample_gap
        proc = np.clip(proc, 0.0, self.max_sight)
        return proc

    def get_max_gap(self, free_ranges):
        longest = 0
        current = 0
        end_idx = 0
        safe_dist = 0.7 + 0.4 * self.current_speed
        self.get_logger().info(f"Safe distance: {safe_dist:.2f}")
        start_idx = 0
        for i in range(len(free_ranges)):
            if free_ranges[i] > safe_dist:
                current += 1
                if current > longest:
                    longest = current
                    end_idx = i + 1
                    start_idx = end_idx - longest
            else:
                current = 0
        return start_idx, end_idx
    
    def normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))


    def compute_gap_steer_and_speed(self, scan_msg, gap_start, gap_end):
        """
        Use the center of the widest gap to define a steering target
        and forward clearance to define a desired speed.

        Changes vs old version:
        - Cruise / upper speed cap now comes from the raceline speed profile (CSV col 5),
            i.e., self.speed_profile[idx] at the nearest waypoint.
        - The gap-based discrete speeds (2/3/4) are still used, but never exceed v_race.

        We *always* compare the gap direction with the raceline direction.
        If the gap is too far from the raceline, we ignore it and stick
        to the raceline, even if the gap is large.
        """
        if scan_msg is None:
            self.obstacle_ahead = False
            self.front_clearance = self.max_sight
            self.raceline_steer = 0.0

            # Fallback: use candidate_speed if speed_profile is not available yet
            v_race = float(self.settings.candidate_speed)
            try:
                # If we have a valid position and speed_profile, use nearest waypoint speed
                cx = self.waypoints[:, 1]
                cy = self.waypoints[:, 2]
                pt = np.array([self.current_x, self.current_y])
                _, _, _, idx = self.get_nearest_point(pt, np.vstack((cx, cy)).T)
                if hasattr(self, "speed_profile"):
                    v_race = float(self.speed_profile[idx])
            except Exception:
                pass

            v_race = float(np.clip(v_race, self.settings.min_speed, self.settings.max_speed))
            return 0.0, v_race

        ranges = np.array(scan_msg.ranges)

        # ---------- 1) Forward clearance ----------
        forward_window = ranges[530:549]  # tune indices if needed
        forward_window = forward_window[np.isfinite(forward_window)]
        if forward_window.size == 0:
            mean_forward = self.max_sight
        else:
            mean_forward = float(np.mean(forward_window))

        self.front_clearance = mean_forward

        # ---------- 2) Raceline steering estimate + raceline speed from CSV ----------
        cx = self.waypoints[:, 1]
        cy = self.waypoints[:, 2]
        pt = np.array([self.current_x, self.current_y])

        _, _, _, idx = self.get_nearest_point(
            pt,
            np.vstack((cx, cy)).T
        )

        raceline_yaw = self.waypoints[idx, 3]
        raceline_steer = self.normalize_angle(raceline_yaw - self.current_yaw)
        raceline_steer = float(np.clip(raceline_steer, self.settings.min_steer, self.settings.max_steer))
        self.raceline_steer = raceline_steer  # store for candidate fan

        # raceline desired speed from CSV col 5 (scaled), clipped to limits
        if hasattr(self, "speed_profile"):
            v_race = float(self.speed_profile[idx])
        else:
            v_race = float(self.settings.candidate_speed)  # fallback

        v_race = float(np.clip(v_race, self.settings.min_speed, self.settings.max_speed))

        # ---------- 3) Decide if we should use gap or raceline ----------
        free_thresh = 4.0   # >4 m ahead = clearly free
        max_dev = math.radians(40.0)  # max allowed deviation from raceline

        # Default: assume no obstacle and stay on raceline
        self.obstacle_ahead = False
        steering_angle = raceline_steer

        if mean_forward < free_thresh:
            # Something is within ~4 m, consider a gap
            self.obstacle_ahead = True

            # Center of widest gap (in downsampled space)
            best_i = 0.5 * (gap_start + gap_end)
            gap_steer = np.deg2rad(best_i * self.downsample_gap / 4.0 - 90.0)

            angle_diff = abs(self.normalize_angle(gap_steer - raceline_steer))

            # Only accept gap steering if it's not too far from raceline
            if angle_diff <= max_dev:
                steering_angle = gap_steer
            else:
                # gap is pointing into a side corridor -> ignore it
                self.obstacle_ahead = False
                steering_angle = raceline_steer

        # ---------- 4) Desired speed ----------
        # Keep your discrete safety speeds, but cap by raceline speed
        if mean_forward < 1.5:
            velocity = 2.0
        elif mean_forward < 2.5:
            velocity = 3.0
        elif mean_forward < free_thresh:
            velocity = 4.0
        else:
            velocity = v_race  # cruise when clear = CSV speed

        # never exceed raceline profile (fairness vs PP)
        velocity = min(float(velocity), float(v_race))
        velocity = min(float(velocity), float(self.settings.max_speed))

        return float(steering_angle), float(velocity)





    # -----------------------------------------------------------
    # Parameters and Visualization
    # -----------------------------------------------------------
    def setup_parameters(self):

        # --- Core MPC / vehicle parameters (matched to mpc.py) ---
        self.declare_parameter("real_environment", False)
        self.declare_parameter("csv_file", "Catalunya_fast")

        self.declare_parameter("candidate_speed", 4.0)

        # Input cost R (accel, steering)
        # (same idea as mpc_config.Rk in mpc.py)
        self.declare_parameter("input_cost_accel", 0.01)
        self.declare_parameter("input_cost_steering", 5.0)

        # Input rate cost Rd
        self.declare_parameter("input_rate_cost_accel", 0.01)
        self.declare_parameter("input_rate_cost_steering", 5.0)

        # State cost Q  (x, y, v, yaw)
        # Match the good tracker: diag([13.5, 13.5, 5.5, 13.0])
        self.declare_parameter("state_cost_x", 13.5)
        self.declare_parameter("state_cost_y", 13.5)
        self.declare_parameter("state_cost_v", 5.5)
        self.declare_parameter("state_cost_yaw", 13.0)

        # Final state cost Qf
        self.declare_parameter("final_state_cost_x", 13.5)
        self.declare_parameter("final_state_cost_y", 13.5)
        self.declare_parameter("final_state_cost_v", 5.5)
        self.declare_parameter("final_state_cost_yaw", 13.0)

        # Dimensions / horizon / time step
        self.declare_parameter("state_dim", 4)
        self.declare_parameter("input_dim", 2)
        self.declare_parameter("horizon", 8)     # TK = 8 like in mpc.py
        self.declare_parameter("search_idx", 20) # N_IND_SEARCH = 20
        self.declare_parameter("dt", 0.1)        # DTK = 0.1
        self.declare_parameter("step_length", 0.2)  # dlk = 0.2 (used below)

        # Vehicle dimensions and limits (copied from mpc.py)
        self.declare_parameter("veh_length", 0.58)
        self.declare_parameter("veh_width", 0.31)
        self.declare_parameter("wheelbase", 0.33)
        self.declare_parameter("min_steer", -0.4189)
        self.declare_parameter("max_steer", 0.4189)
        self.declare_parameter("max_steer_rate", float(np.deg2rad(180.0)))
        self.declare_parameter("max_speed", 14.5)   # same as mpc_config.MAX_SPEED
        self.declare_parameter("min_speed", 3.777)
        self.declare_parameter("max_accel", 5.0)

        # --- Obstacle-avoidance / gap follow hyperparams (keep these) ---
        self.declare_parameter("downsampling", 10)
        self.declare_parameter("max_sight", 5.0)
        self.declare_parameter("gap_threshold", 15.0)
        self.declare_parameter("pred_history", 20)
        # safety distance to keep avoidance active
        self.declare_parameter("max_gap_safe_dist", 1.8)

        self.declare_parameter("speed_scale", 1.0)       # match PP (e.g., 1.2) if you want
        self.declare_parameter("use_csv_speed", True)



    def show_waypoints(self):
        self.wpts_marker.points = []
        self.wpts_marker.header.frame_id = '/map'
        self.wpts_marker.type = Marker.POINTS
        self.wpts_marker.color.g = 0.75
        self.wpts_marker.color.a = 1.0
        self.wpts_marker.scale.x = 0.05
        self.wpts_marker.scale.y = 0.05
        self.wpts_marker.id = 0
        for i in range(self.waypoints.shape[0]):
            pt = Point(x=self.waypoints[i, 1],
                       y=self.waypoints[i, 2],
                       z=0.1)
            self.wpts_marker.points.append(pt)
        self.wpts_pub.publish(self.wpts_marker)

    def show_reference_traj(self, ref_traj):
        self.ref_traj_marker.points = []
        self.ref_traj_marker.header.frame_id = '/map'
        self.ref_traj_marker.type = Marker.LINE_STRIP
        self.ref_traj_marker.color.b = 0.75
        self.ref_traj_marker.color.a = 1.0
        self.ref_traj_marker.scale.x = 0.08
        self.ref_traj_marker.scale.y = 0.08
        self.ref_traj_marker.id = 0
        for i in range(ref_traj.shape[1]):
            pt = Point(x=ref_traj[0, i],
                       y=ref_traj[1, i],
                       z=0.2)
            self.ref_traj_marker.points.append(pt)
        self.ref_traj_pub.publish(self.ref_traj_marker)

    def show_predicted_paths(self, global_paths, best_idx):
        """
        Visualize all candidate trajectories as a fan of red lines.
        The best one is drawn thicker and in a different color.
        """
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for k, path in enumerate(global_paths):
            marker = Marker()
            marker.header.frame_id = '/map'
            marker.header.stamp = now
            marker.ns = "mpc_candidates"
            marker.id = k
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Best path: thicker and different color
            if k == best_idx:
                marker.scale.x = 0.08
                # chosen trajectory color (e.g., bright blue)
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
            else:
                marker.scale.x = 0.03
                # keep other predicted trajectories as your original red
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.7

            marker.points = []
            for i in range(path.shape[1]):
                pt = Point()
                pt.x = float(path[0, i])
                pt.y = float(path[1, i])
                pt.z = 0.2
                marker.points.append(pt)

            marker_array.markers.append(marker)

        self.pred_path_pub.publish(marker_array)



def main(args=None):
    rclpy.init(args=args)
    print("Fan-of-trajectories MPC Node Initialized")
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
