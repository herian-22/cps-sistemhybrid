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
        # Note: speed is kept for compatibility but not used for custom_obstacles
        # as they are static. The original update method is removed.

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
        self.robot_z       = 0.0     # World forward coordinate
        self.robot_y_offset= 0.0     # Bobbing
        self.bounce_velocity= 0.0    # For elastic collision bounce

        # ── Walking gait parameters ──────────────────────────────────────────
        self.walking_phase     = 0.0
        self.walking_speed     = 5.0
        self.walking_amplitude = 30.0

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
        # raw_signal: noisy continuous reading (sine + noise)
        # filtered_signal: low-pass filtered version
        # quantized_event: 0 or 1 (thresholded discrete output)
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
            # self.obstacles[0].z = 200.0 # Removed as self.obstacles is removed
            self.distance = 100.0

    # ── Main tick ─────────────────────────────────────────────────────────────
    def update(self):
        if not self.is_running: return
        self.time_elapsed += self.DT

        # In AUTO: robot only moves forward if mode is WALKING.
        # In MANUAL: user controls nothing since it's world physics; we just walk forward infinitely
        #            but we will use AUTO mechanics to actually advance. So WALKING = advance.
        if self.mode == SystemMode.WALKING:
            self._walking_dynamics()
        else:
            self._evasive_dynamics()

        # Apply and decay bounce momentum (elastic collision physics)
        if abs(self.bounce_velocity) > 0.1:
            self.robot_z += self.bounce_velocity * self.DT * 50.0
            self.bounce_velocity *= 0.90  # Friction decay

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

    # ── Signal processing: continuous → discrete ──────────────────────────────
    def _process_signal(self, base_dist: float):
        """
        Simulates a real sensor pipeline:
          1. Raw signal = base + noise  (continuous, noisy)
          2. Filtered = IIR low-pass filter  (smoothed continuous)
          3. Quantized event = threshold guard  (discrete 0/1)
        The discrete guard only fires when the FILTERED value crosses the threshold.
        """
        # 1. Inject noise
        noise = random.gauss(0, 1.5)
        self.raw_signal = base_dist + noise

        # 2. IIR low-pass filter: y[n] = α·x[n] + (1-α)·y[n-1]
        self.filtered_signal = (self.filter_alpha * self.raw_signal
                                + (1 - self.filter_alpha) * self.filtered_signal)

        # 3. Threshold quantisation (with small dead-band to avoid chatter)
        # Previous event is remembered (hysteresis) in normal mode
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

        # Check collision with custom obstacles
        # Simple bounding box check
        custom_danger = False
        danger_id = None
        for obs in self.custom_obstacles:
            # Distance <= 25cm triggers evasion
            if 0 < (obs.z - self.robot_z - obs.depth/2) < 25 and abs(obs.x) < 50 + obs.width/2:
                custom_danger = True
                danger_id = obs.id
                break

        if custom_danger:
            self.quantized_event = 1
            self.highlighted_obs_id = danger_id
        elif not custom_danger and self.zeno_mode:
            # Revert to normal sensor if custom obstacle passed
            pass # keep standard quantized_event check
        else:
            self.highlighted_obs_id = None

        if self.mode == SystemMode.WALKING and self.quantized_event == 1:
            self.mode = SystemMode.EVASIVE
            self._reset_map()
            self.transition_count += 1; self.just_transitioned = True
        elif self.mode == SystemMode.EVASIVE and self.quantized_event == 0:
            self.mode = SystemMode.WALKING
            self.transition_count += 1; self.just_transitioned = True
        if self.mode is old:
            self.just_transitioned = False

    def _check_transitions(self):
        """Legacy: used by Zeno mode for rapid re-checking each tick."""
        pass  # Delegate to _process_signal → _check_transitions_filtered

    def _auto_calculate_distance(self):
        """Calculate physical distance to the nearest forward obstacle globally."""
        closest = 9999.0
        for obs in self.custom_obstacles:
            if obs.z > self.robot_z - (obs.depth/2):
                # Physical horizontal overlap check
                if abs(obs.x) < 50 + obs.width/2:
                    d = obs.z - self.robot_z - (obs.depth/2)
                    if d < closest: closest = d
        
        # Hybrid Automaton Rule: Elastic Collision
        if closest <= 0:
            # v+ = -e * v-
            # Robot snaps to surface
            self.robot_z += (closest - 0.1)
            # Fling backward proportionally to walking_speed
            e = 0.6  # Coefficient of restitution
            self.bounce_velocity = -e * (self.walking_speed * 15.0)
            self.distance = 0.0
        else:
            self.distance = max(0.0, closest)

    def _reset_map(self):
        self.walking_phase += math.pi / 2
        for i in range(4): self.servo_angles[i] += 15.0

    def _walking_dynamics(self):
        # Progress forward in world
        self.robot_z += (self.walking_speed / 5.0) * 10.0 * self.DT

        self.walking_phase += self.walking_speed * self.DT
        a, ph = self.walking_amplitude, self.walking_phase
        self.servo_angles[0] = 90 + a*math.sin(ph)
        self.servo_angles[1] = 90 + a*math.sin(ph + math.pi)
        self.servo_angles[2] = 90 + a*math.sin(ph)
        self.servo_angles[3] = 90 + a*math.sin(ph + math.pi)

        # Bobbing
        self.robot_y_offset = abs(math.sin(ph)) * 3.0

    def _evasive_dynamics(self):
        targets = [45.0, 135.0, 45.0, 135.0]
        for i in range(4):
            self.servo_angles[i] += (targets[i] - self.servo_angles[i]) * 0.15

    # ── Reset ─────────────────────────────────────────────────────────────────
    def reset(self):
        self.mode = SystemMode.WALKING
        self.distance = 100.0
        self.servo_angles = [90.0]*4
        self.time_elapsed = 0.0
        self.walking_phase = 0.0
        self.transition_count = 0
        self.just_transitioned = False
        self.auto_backoff = False
        self.filtered_signal = 100.0
        self.raw_signal = 100.0
        self.quantized_event = 0
        self.highlighted_obs_id = None
        self.robot_z = 0.0
        self.bounce_velocity = 0.0
