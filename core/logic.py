import math
import random
from enum import Enum

class SystemMode(Enum):
    WALKING = "WALKING"
    EVASIVE = "EVASIVE"

class ControlMode(Enum):
    AUTO   = "AUTO"
    MANUAL = "MANUAL"

# ── Obstacle definition ───────────────────────────────────────────────────────
class Obstacle:
    """A single moving obstacle in the 3D environment."""
    def __init__(self, x, base_z, speed, width, height, depth, id=""):
        self.id      = id
        self.x       = x          # lateral offset
        self.z       = float(base_z)
        self.width   = width
        self.height  = height
        self.depth   = depth

class HybridSystemLogic:
    """
    Hybrid Automaton: 4-servo walking robot.
    Features:
    - MANUAL / AUTO control modes
    - Multiple independently-moving obstacles
    - Continuous sensor signal processed through a discrete threshold guard
    - Zeno simulation mode
    """
    THRESHOLD_DANGER = 20.0
    THRESHOLD_SAFE   = 30.0
    DT               = 0.05  # seconds per tick

    def __init__(self):
        # ── Discrete state ──────────────────────────────────────────────────
        self.mode               = SystemMode.WALKING
        self.control            = ControlMode.MANUAL
        self.just_transitioned  = False
        self.transition_count   = 0

        # ── Continuous state ─────────────────────────────────────────────────
        self.distance      = 100.0   # cm, computed from world geometry
        self.servo_angles  = [90.0]*4
        self.femur_len     = 50.0
        self.tibia_len     = 50.0
        self.time_elapsed  = 0.0
        self.is_running    = False
        self.use_ik        = True    # Toggle Inverse Kinematics vs straight rigid lines

        # ── World physical state ─────────────────────────────────────────────
        self.robot_z        = 0.0     # World forward coordinate
        self.robot_y_offset = 0.0     # Bobbing
        self.bounce_velocity= 0.0     # For elastic collision bounce

        # ── Walking gait parameters ──────────────────────────────────────────
        self.walking_phase     = 0.0
        self.walking_speed     = 5.0
        self.walking_amplitude = 30.0

        # ── Hardware Simulation (Realistic Digital Twin) ─────────────────────
        self.mcu_type     = "ESP32-WROOM-32"
        self.sensor_type  = "HC-SR04 Ultrasonik"
        self.servo_type   = "MG996R High Torque"
        self.mcu_status   = "ACTIVE"
        self.battery_level= 98.0  # % Lipo Battery
        self.sensor_min   = 2.0  # HC-SR04 min 2cm
        self.sensor_max   = 400.0 # HC-SR04 max 400cm

        # ── AUTO drive state ─────────────────────────────────────────────────
        self.auto_approach_speed = 0.4
        self.auto_backoff        = False
        self.auto_backoff_dist   = 65.0

        # ── Zeno mode ────────────────────────────────────────────────────────
        self.zeno_mode = False

        # ── Map layout (Gazebo-style structural elements) ────────────────────
        self.custom_obstacles = [
            Obstacle(-150, 400, 0.0, 80, 120, 80, "pillar_1"),
            Obstacle( 150, 400, 0.0, 80, 120, 80, "pillar_2"),
            Obstacle(   0, 800, 0.0, 100,  80, 40, "box_1"),
            Obstacle( 200, 1200, 0.0, 150,  50, 60, "ramp_1"),
            Obstacle(-250, 1600, 0.0, 200, 200, 60, "wall_1"),
            Obstacle(   0, 2500, 0.0, 600, 300, 100, "end_wall"),
        ]
        self.highlighted_obs_id = None

        # ── Signal processing state (continuous → discrete) ──────────────────
        self.raw_signal       = 100.0
        self.filtered_signal  = 100.0
        self.filter_alpha     = 0.15   # IIR low-pass coefficient
        self.quantized_event  = 0      # 0 = SAFE, 1 = DANGER

        # History buffers for analytics
        self.history_size    = 100
        self.history_dist    = [100.0]*100
        self.history_servos  = [[90.0]*100 for _ in range(4)]
        self.history_raw     = [100.0]*100
        self.history_filtered= [100.0]*100
        self.history_mode    = [1]*100     # 1 = WALKING, 0 = EVASIVE
        self.history_z       = [0.0]*100   # Trajectory tracking

    def add_custom_obstacle(self, x, z, w, h, d):
        obs_id = f"custom_{len(self.custom_obstacles)}"
        self.custom_obstacles.append(Obstacle(x, z, 0.0, w, h, d, obs_id))

    def remove_custom_obstacle(self, idx):
        if 0 <= idx < len(self.custom_obstacles):
            self.custom_obstacles.pop(idx)

    # ── External inputs ───────────────────────────────────────────────────────
    def set_distance(self, d: float):
        """Deprecated: distance is now driven purely by world mechanics."""
        pass

    def set_control(self, mode: ControlMode):
        self.control = mode
        if mode == ControlMode.AUTO:
            self.distance = 100.0

    # ── Main tick ─────────────────────────────────────────────────────────────
    def update(self):
        if not self.is_running: return
        self.time_elapsed += self.DT

        # Reality check: Battery drain
        self.battery_level = max(0.0, self.battery_level - 0.001)
        if self.battery_level < 15: self.mcu_status = "LOW BATT"
        if self.battery_level <= 0:
            self.mcu_status = "CRITICAL (OFF)"
            self.is_running = False
            return

        if self.mode == SystemMode.WALKING:
            self._walking_dynamics()
        else:
            self._evasive_dynamics()

        # Apply and decay bounce momentum (elastic collision physics)
        if abs(self.bounce_velocity) > 0.1:
            self.robot_z += self.bounce_velocity * self.DT * 50.0
            self.bounce_velocity *= 0.88  # Friction decay

        # Compute physical distance to next obstacle
        self._auto_calculate_distance()

        # Process signal (continuous → filtered → discrete event)
        self._process_signal(self.distance)

        if self.zeno_mode:
            self._check_transitions()

        # Update rolling buffers
        self.history_dist.pop(0); self.history_dist.append(self.distance)
        self.history_raw.pop(0);  self.history_raw.append(self.raw_signal)
        self.history_filtered.pop(0); self.history_filtered.append(self.filtered_signal)
        for i in range(4):
            self.history_servos[i].pop(0)
            self.history_servos[i].append(self.servo_angles[i])
        
        self.history_mode.pop(0)
        self.history_mode.append(1 if self.mode == SystemMode.WALKING else 0)
        self.history_z.pop(0)
        self.history_z.append(self.robot_z)

    # ── Signal processing: continuous → discrete ──────────────────────────────
    def _process_signal(self, base_dist: float):
        """
        Simulates a real sensor pipeline:
          1. Raw signal = base + noise  (continuous, noisy)
          2. Filtered = IIR low-pass filter  (smoothed continuous)
          3. Quantized event = threshold guard  (discrete 0/1)
        """
        # 1. Inject noise
        noise = random.gauss(0, 1.5)
        # Reality check: HC-SR04 has physical range limits
        clamped_base = max(self.sensor_min, min(self.sensor_max, base_dist))
        self.raw_signal = clamped_base + noise

        # 2. IIR low-pass filter: y[n] = α·x[n] + (1-α)·y[n-1]
        self.filtered_signal = (self.filter_alpha * self.raw_signal
                                + (1 - self.filter_alpha) * self.filtered_signal)

        # 3. Threshold quantisation (with hysteresis dead-band)
        if self.zeno_mode:
            # Collapse hysteresis – triggers Zeno chattering
            self.quantized_event = 1 if self.filtered_signal <= 25.0 else 0
        else:
            if self.quantized_event == 0 and self.filtered_signal <= self.THRESHOLD_DANGER:
                self.quantized_event = 1
            elif self.quantized_event == 1 and self.filtered_signal >= self.THRESHOLD_SAFE:
                self.quantized_event = 0

        # 4. Feed discrete event into the hybrid automaton guard
        self._check_transitions_filtered()

    def _check_transitions_filtered(self):
        """Guard uses filtered/quantized signal, NOT noisy raw."""
        old = self.mode

        # Check collision with custom obstacles (bounding box check)
        custom_danger = False
        danger_id = None
        for obs in self.custom_obstacles:
            front_dist = obs.z - self.robot_z - obs.depth / 2
            # Bug fix: check front_dist >= 0 to avoid behind-obstacle triggers
            if 0 <= front_dist < 25 and abs(obs.x) < 50 + obs.width / 2:
                custom_danger = True
                danger_id = obs.id
                break

        if custom_danger:
            self.quantized_event = 1
            self.highlighted_obs_id = danger_id
        else:
            # Bug fix: always clear highlight when no custom danger
            self.highlighted_obs_id = None

        if self.mode == SystemMode.WALKING and self.quantized_event == 1:
            self.mode = SystemMode.EVASIVE
            self._reset_map()
            self.transition_count += 1
            self.just_transitioned = True
        elif self.mode == SystemMode.EVASIVE and self.quantized_event == 0:
            self.mode = SystemMode.WALKING
            self.transition_count += 1
            self.just_transitioned = True

        if self.mode is old:
            self.just_transitioned = False

    def _check_transitions(self):
        """Legacy: used by Zeno mode for rapid re-checking each tick."""
        pass  # Delegate to _process_signal → _check_transitions_filtered

    def _auto_calculate_distance(self):
        """Calculate physical distance to the nearest forward obstacle globally."""
        closest = 9999.0
        for obs in self.custom_obstacles:
            front_edge = obs.z - obs.depth / 2
            # Only consider obstacles ahead of the robot
            if obs.z > self.robot_z:
                # Physical horizontal overlap check
                if abs(obs.x) < 50 + obs.width / 2:
                    d = front_edge - self.robot_z
                    if d < closest:
                        closest = d

        # Bug fix: improved elastic collision – snap precisely to surface
        if closest <= 0:
            # Snap robot back to the surface (don't let it pass through)
            self.robot_z += closest - 0.5
            # Elastic bounce: v+ = -e * v-
            e = 0.55  # Coefficient of restitution
            self.bounce_velocity = -e * (self.walking_speed * 15.0)
            self.distance = 0.5
        else:
            self.distance = max(0.0, min(closest, 999.0))

    def _reset_map(self):
        self.walking_phase += math.pi / 2
        for i in range(4): self.servo_angles[i] += 15.0

    def _walking_dynamics(self):
        # Progress forward in world
        self.robot_z += (self.walking_speed / 5.0) * 10.0 * self.DT

        self.walking_phase += self.walking_speed * self.DT
        a, ph = self.walking_amplitude, self.walking_phase
        self.servo_angles[0] = 90 + a * math.sin(ph)
        self.servo_angles[1] = 90 + a * math.sin(ph + math.pi)
        self.servo_angles[2] = 90 + a * math.sin(ph)
        self.servo_angles[3] = 90 + a * math.sin(ph + math.pi)

        # Bobbing
        self.robot_y_offset = abs(math.sin(ph)) * 3.0

    def _evasive_dynamics(self):
        """Bug fix: robot now also moves backward while evading."""
        targets = [45.0, 135.0, 45.0, 135.0]
        for i in range(4):
            self.servo_angles[i] += (targets[i] - self.servo_angles[i]) * 0.15

        # Back off slowly while in evasive mode (if bounce already decayed)
        if abs(self.bounce_velocity) < 0.5:
            self.robot_z -= (self.walking_speed / 5.0) * 3.0 * self.DT
        self.robot_y_offset = 0.0

    # ── Reset ─────────────────────────────────────────────────────────────────
    def reset(self):
        self.mode = SystemMode.WALKING
        self.distance = 100.0
        self.servo_angles = [90.0]*4
        self.time_elapsed = 0.0
        self.walking_phase = 0.0
        self.robot_y_offset = 0.0
        self.transition_count = 0
        self.just_transitioned = False
        self.auto_backoff = False
        self.filtered_signal = 100.0
        self.raw_signal = 100.0
        self.quantized_event = 0
        self.highlighted_obs_id = None
        self.robot_z = 0.0
        self.bounce_velocity = 0.0
        # Bug fix: also reset history buffers so plots clear on reset
        self.history_dist     = [100.0]*self.history_size
        self.history_raw      = [100.0]*self.history_size
        self.history_filtered = [100.0]*self.history_size
        self.history_servos   = [[90.0]*self.history_size for _ in range(4)]
        self.history_mode     = [1]*self.history_size
        self.history_z        = [0.0]*self.history_size
        
        # Reset hardware status
        self.battery_level    = 98
        self.mcu_status       = "ACTIVE"
