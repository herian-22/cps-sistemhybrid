import math
import time
import random
from enum import Enum
from PySide6.QtCore import QThread, Signal, QObject
from core.pathfinding import OccupancyGrid, a_star_search
from core.logger import log

class SystemMode(Enum):
    WALKING    = "WALKING"
    HALTING    = "HALTING"
    SCANNING   = "SCANNING"
    PLANNING   = "PLANNING"
    NAVIGATING = "NAVIGATING"
    FOLLOWING  = "FOLLOWING"

class ControlMode(Enum):
    AUTO   = "AUTO"
    MANUAL = "MANUAL"

class Obstacle:
    def __init__(self, x, base_z, speed, width, height, depth, id=""):
        self.id      = id
        self.x       = x
        self.z       = float(base_z)
        self.width   = width
        self.height  = height
        self.depth   = depth

# ── Concurrency Worker ───────────────────────────────────────────────────────
class ComputationWorker(QObject):
    finished = Signal(object)
    
    def run_a_star(self, grid, start, goal):
        log(f"A* worker started: {start} -> {goal}", "DEBUG")
        path = a_star_search(grid, start, goal)
        if path:
            log(f"A* path found with {len(path)} nodes.")
        else:
            log("A* failed to find a path.", "WARNING")
        self.finished.emit(path)

# ── Physics Engine ───────────────────────────────────────────────────────────
class PhysicsEngine:
    def __init__(self):
        self.robot_x = 0.0
        self.robot_z = 0.0
        self.robot_rotation = 0.0
        self.robot_y_offset = 0.0
        self.bounce_velocity = 0.0
        self.custom_obstacles = [Obstacle(0, 1000, 0.0, 100, 80, 100, "box_1")]
        self.highlighted_obs_id = None
        
    def get_min_distance(self, angles=[0.0, -30.0, 30.0]):
        """Returns the minimum distance from multiple raycast angles."""
        distances = [self.raycast_distance(a) for a in angles]
        return min(distances)

    def raycast_distance(self, angle_offset_deg=0.0):
        rx, rz = self.robot_x, self.robot_z
        theta = math.radians(self.robot_rotation + angle_offset_deg)
        dx, dz = math.sin(theta), math.cos(theta)
        min_t = 2500.0
        hit_obs = None
        
        for obs in self.custom_obstacles:
            min_x, max_x = obs.x - obs.width/2, obs.x + obs.width/2
            min_z, max_z = obs.z - obs.depth/2, obs.z + obs.depth/2
            t_min, t_max = 0.0, float('inf')
            
            if abs(dx) < 1e-6:
                if rx < min_x or rx > max_x: continue
            else:
                tx1, tx2 = (min_x - rx) / dx, (max_x - rx) / dx
                t_min, t_max = max(t_min, min(tx1, tx2)), min(t_max, max(tx1, tx2))
                
            if abs(dz) < 1e-6:
                if rz < min_z or rz > max_z: continue
            else:
                tz1, tz2 = (min_z - rz) / dz, (max_z - rz) / dz
                t_min, t_max = max(t_min, min(tz1, tz2)), min(t_max, max(tz1, tz2))
                
            if t_min <= t_max and t_min > 0 < min_t:
                if t_min < min_t:
                    min_t, hit_obs = t_min, obs
                    
        self.highlighted_obs_id = hit_obs.id if hit_obs else None
        return min_t

    def check_collision(self, next_x, next_z):
        """Checks if the next position is inside any obstacle."""
        for obs in self.custom_obstacles:
            # Simple axial bounding box check
            margin = 30.0 # Safety buffer
            min_x, max_x = obs.x - obs.width/2 - margin, obs.x + obs.width/2 + margin
            min_z, max_z = obs.z - obs.depth/2 - margin, obs.z + obs.depth/2 + margin
            if min_x < next_x < max_x and min_z < next_z < max_z:
                return True
        return False

    def apply_dynamics(self, dt):
        if abs(self.bounce_velocity) > 0.1:
            next_z = self.robot_z + self.bounce_velocity * dt * 50.0
            if not self.check_collision(self.robot_x, next_z):
                self.robot_z = next_z
            else:
                self.bounce_velocity = 0 # Stop bouncing on hit
            self.bounce_velocity *= 0.90

# ── Performance Metrics ───────────────────────────────────────────────────────
class MetricsTracker:
    def __init__(self):
        self.score = 100.0
        self.collisions = 0
        self.start_time = time.time()
        self.energy_used = 0.0
        self.distance_traveled = 0.0
        self.last_pos = (0.0, 0.0)
        
    def log_collision(self):
        self.collisions += 1
        self.score = max(0.0, self.score - 5.0)
        log(f"Collision Registered! Penalty applied. Score: {self.score:.1f}", "WARNING")
        
    def update(self, current_pos, servos_moved=0):
        # Update distance
        d = math.hypot(current_pos[0] - self.last_pos[0], current_pos[1] - self.last_pos[1])
        self.distance_traveled += d
        self.last_pos = current_pos
        
        # Energy proxy (based on servo travel)
        self.energy_used += servos_moved * 0.01
        
        # Time penalty (if over a certain duration)
        raw_time = time.time() - self.start_time
        if raw_time > 60: # Penalty after 1 minute
            self.score = max(0, self.score - 0.01) # Slow drain

# ── Kinematics ───────────────────────────────────────────────────────────────
class RobotKinematics:
    def __init__(self):
        self.servo_angles = [90.0]*8
        self.femur_len = 50.0
        self.tibia_len = 50.0
        self.walking_phase = 0.0
        self.walking_speed = 5.0
        self.walking_amplitude = 30.0

    def solve_ik(self, y: float, z: float):
        L1, L2 = self.femur_len, self.tibia_len
        D = math.sqrt(y**2 + z**2)
        if D > (L1 + L2 - 0.1):
            scale = (L1 + L2 - 0.1) / max(0.1, D)
            y, z = y * scale, z * scale
            D = L1 + L2 - 0.1
        val = max(-1.0, min(1.0, (D**2 - L1**2 - L2**2) / (2 * L1 * L2)))
        q2 = math.acos(val)
        gamma = math.atan2(y, z)
        alpha = math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
        q1 = gamma - alpha
        return math.degrees(q1) + 90, math.degrees(q2) + 90

    def update_gait(self, dt, moving=True):
        if moving:
            self.walking_phase += self.walking_speed * dt
        ph = self.walking_phase
        a = self.walking_amplitude
        phases = [ph, ph + math.pi, ph + math.pi, ph]
        base_y = -(self.femur_len + self.tibia_len - 15.0)
        
        for i in range(4):
            sy = math.sin(phases[i])
            ty = base_y + (a * 0.8 * sy if sy > 0 and moving else 0)
            tz = -a * 1.5 * math.cos(phases[i]) if moving else 0
            q1, q2 = self.solve_ik(ty, tz)
            self.servo_angles[i*2], self.servo_angles[i*2+1] = q1, q2
        return abs(math.sin(ph)) * (a * 0.15) if moving else 0

# ── State Machine ───────────────────────────────────────────────────────────
class RobotStateMachine:
    def __init__(self):
        self.mode = SystemMode.WALKING
        self.control = ControlMode.MANUAL
        self.raw_signal = 100.0
        self.filtered_signal = 100.0
        self.quantized_event = 0
        self.hysteresis_threshold = 25.0
        self.hysteresis_gap = 10.0
        self.scan_timer = 0.0
        self.scan_interval = 8.0 # Proactive scan every 8s

    def process_sensors(self, dist, noise_lvl):
        noise = random.uniform(-noise_lvl, noise_lvl)
        self.raw_signal = dist + noise
        alpha = 0.2
        self.filtered_signal = alpha * self.raw_signal + (1 - alpha) * self.filtered_signal
        
        lower = self.hysteresis_threshold - self.hysteresis_gap / 2
        
        # Proactive scan trigger
        if self.mode == SystemMode.WALKING:
            self.scan_timer += 0.05 # DT
            if self.scan_timer > self.scan_interval:
                log("Triggering proactive scan...")
                self.mode = SystemMode.HALTING
                self.scan_timer = 0.0
                return True

        if self.mode == SystemMode.WALKING and self.filtered_signal < lower:
            log(f"Obstacle detected! Signal: {self.filtered_signal:.1f}. Halting.", "WARNING")
            self.mode = SystemMode.HALTING
            self.scan_timer = 0.0
            return True
        return False

# ── Main Controller ──────────────────────────────────────────────────────────
class HybridSystemLogic(QObject):
    DT = 0.05
    
    def __init__(self):
        super().__init__()
        self.physics = PhysicsEngine()
        self.kinematics = RobotKinematics()
        self.state = RobotStateMachine()
        self.metrics = MetricsTracker()
        
        self.occupancy_grid = OccupancyGrid()
        self.planned_path = []
        self.nav_target = None
        self.is_running = False
        self.time_elapsed = 0.0
        self.zeno_mode = False
        self.use_ik = True
        self.transition_count = 0
        self.just_transitioned = False
        
        self.scan_sweep_angle = -45.0
        self.scan_dir = 1
        self.planning_active = False
        
        # IR Sensors (Simulated)
        self.ir_sensors = [0.0, 0.0, 0.0] # Left, Center, Right
        
        log("System initialized and ready.")
        
        # History for Analytics
        self.history_dist = [100.0] * 100
        self.history_raw = [100.0] * 100
        self.history_filtered = [100.0] * 100
        self.history_servos = [[] for _ in range(8)]
        
        # Threading for A*
        self.worker_thread = QThread()
        self.worker = ComputationWorker()
        self.worker.moveToThread(self.worker_thread)
        self.worker.finished.connect(self._on_path_found)
        self.worker_thread.start()

    def _on_path_found(self, path):
        self.planning_active = False
        if path and len(path) > 1:
            self.planned_path = path[1:]
            self.nav_target = self.planned_path.pop(0)
            self.state.mode = SystemMode.NAVIGATING
        else:
            self.physics.robot_rotation += 45.0
            self.state.mode = SystemMode.WALKING

    def _update_ir_sensors(self):
        """Simulates 3 IR sensors on the bottom-front of the robot."""
        # Sensor positions relative to robot center (in world units)
        offsets = [(-100, 200), (0, 200), (100, 200)] # Left, Center, Right
        for i, (ox, oz) in enumerate(offsets):
            # Rotate offset by robot rotation
            rad = math.radians(self.physics.robot_rotation)
            sx = self.physics.robot_x + ox * math.cos(rad) + oz * math.sin(rad)
            sz = self.physics.robot_z - ox * math.sin(rad) + oz * math.cos(rad)
            
            # Check grid for "line" (we'll define line as value 0.8)
            val = self.occupancy_grid.get_value(sx, sz)
            self.ir_sensors[i] = 1.0 if val > 0.7 else 0.0

    def update(self):
        if not self.is_running: return
        self.time_elapsed += self.DT
        self.physics.apply_dynamics(self.DT)
        
        # Update Sensors & Metrics
        self._update_ir_sensors()
        dist = self.physics.get_min_distance()
        self.metrics.update((self.physics.robot_x, self.physics.robot_z))
        
        if self.state.process_sensors(dist, 3.0 if self.zeno_mode else 1.5):
            self.transition_count += 1
            self.just_transitioned = True
            log(f"State transition: {self.state.mode.value}")
        
        if self.state.mode == SystemMode.WALKING:
            self._handle_walking()
        elif self.state.mode == SystemMode.HALTING:
            self.state.mode = SystemMode.SCANNING
            self.scan_sweep_angle, self.scan_dir = -45.0, 1
        elif self.state.mode == SystemMode.SCANNING:
            self._handle_scanning()
        elif self.state.mode == SystemMode.PLANNING:
            if getattr(self, "planning_active", False):
                return
            # ... (planning logic)
            self.planning_active = True # Already in file
            # ...
        elif self.state.mode == SystemMode.NAVIGATING:
            self._handle_navigating()
        elif getattr(SystemMode, 'FOLLOWING', None) and self.state.mode == SystemMode.FOLLOWING:
            self._handle_following()

        self._update_history()

    def _handle_walking(self):
        moving = True
        if self.state.control == ControlMode.AUTO and self.physics.raycast_distance() < 40.0:
            moving = False
        
        if moving:
            spd = self.kinematics.walking_speed * 10 * self.DT
            nx = self.physics.robot_x + spd * math.sin(math.radians(self.physics.robot_rotation))
            nz = self.physics.robot_z + spd * math.cos(math.radians(self.physics.robot_rotation))
            
            if not self.physics.check_collision(nx, nz):
                self.physics.robot_x, self.physics.robot_z = nx, nz
            else:
                self.metrics.log_collision()
                moving = False
            
        self.physics.robot_y_offset = self.kinematics.update_gait(self.DT, moving)

    def _handle_following(self):
        """Logic for Legged Line Follower."""
        l, c, r = self.ir_sensors
        
        # Simple proportional control
        turn_speed = 0.0
        if l > 0.5: turn_speed = -60.0 # Turn left
        elif r > 0.5: turn_speed = 60.0 # Turn right
        
        self.physics.robot_rotation += turn_speed * self.DT
        
        # Move forward if center is on line or we are turning
        moving = True
        spd = self.kinematics.walking_speed * 8 * self.DT
        nx = self.physics.robot_x + spd * math.sin(math.radians(self.physics.robot_rotation))
        nz = self.physics.robot_z + spd * math.cos(math.radians(self.physics.robot_rotation))
        
        if not self.physics.check_collision(nx, nz):
            self.physics.robot_x, self.physics.robot_z = nx, nz
        else:
            self.metrics.log_collision()
            moving = False
            
        self.physics.robot_y_offset = self.kinematics.update_gait(self.DT, moving)

    def _handle_scanning(self):
        self.scan_sweep_angle += self.scan_dir * 150.0 * self.DT
        if self.scan_sweep_angle > 45.0: self.scan_dir = -1
        if self.scan_sweep_angle < -45.0 and self.scan_dir == -1:
            self.state.mode = SystemMode.PLANNING
            
        d = self.physics.raycast_distance(self.scan_sweep_angle)
        ang = math.radians(self.physics.robot_rotation + self.scan_sweep_angle)
        if d < 2500.0:
            hx, hz = self.physics.robot_x + d*math.sin(ang), self.physics.robot_z + d*math.cos(ang)
            self.occupancy_grid.update_raycast(self.physics.robot_x, self.physics.robot_z, hx, hz, True)
        else:
            ex, ez = self.physics.robot_x + 1500*math.sin(ang), self.physics.robot_z + 1500*math.cos(ang)
            self.occupancy_grid.update_raycast(self.physics.robot_x, self.physics.robot_z, ex, ez, False)

    def _handle_navigating(self):
        if self.nav_target:
            tx, tz = self.nav_target
            dist_to_target = math.hypot(tx - self.physics.robot_x, tz - self.physics.robot_z)
            
            # Rotation
            ang = math.degrees(math.atan2(tx - self.physics.robot_x, tz - self.physics.robot_z))
            diff = (ang - self.physics.robot_rotation + 180) % 360 - 180
            self.physics.robot_rotation += diff * 5.0 * self.DT
            
            # Movement
            if abs(diff) < 20: # Only move forward if mostly facing the target
                spd = self.kinematics.walking_speed * 10 * self.DT
                self.physics.robot_x += spd * math.sin(math.radians(self.physics.robot_rotation))
                self.physics.robot_z += spd * math.cos(math.radians(self.physics.robot_rotation))
            
            if dist_to_target < 50:
                self.nav_target = self.planned_path.pop(0) if self.planned_path else None
                if not self.nav_target: 
                    log("Navigation goal reached.")
                    self.state.mode = SystemMode.WALKING
                    self.state.scan_timer = 0.0 # Reset scan timer to wait before next scan
        
        self.physics.robot_y_offset = self.kinematics.update_gait(self.DT, True)

    def _update_history(self):
        self.history_dist.pop(0); self.history_dist.append(self.state.filtered_signal)
        self.history_raw.append(self.state.raw_signal); self.history_filtered.append(self.state.filtered_signal)
        if len(self.history_raw) > 100: self.history_raw.pop(0); self.history_filtered.pop(0)
        for i in range(8):
            self.history_servos[i].append(self.kinematics.servo_angles[i])
            if len(self.history_servos[i]) > 100: self.history_servos[i].pop(0)

    # UI Interaction Methods
    def set_distance(self, d):
        if self.state.control == ControlMode.MANUAL:
            self.state.raw_signal = float(d) # Simulate manual distance input

    def set_control(self, mode):
        self.state.control = mode

    def add_custom_obstacle(self, x, z, w, h, d):
        obs_id = f"custom_{len(self.physics.custom_obstacles)}"
        self.physics.custom_obstacles.append(Obstacle(x, z, 0.0, w, h, d, obs_id))

    def remove_custom_obstacle(self, idx):
        if 0 <= idx < len(self.physics.custom_obstacles):
            self.physics.custom_obstacles.pop(idx)

    # Proxy properties for UI compatibility
    @property
    def mode(self): return self.state.mode
    @property
    def control(self): return self.state.control
    @property
    def robot_x(self): return self.physics.robot_x
    @robot_x.setter
    def robot_x(self, v): self.physics.robot_x = v
    @property
    def robot_z(self): return self.physics.robot_z
    @robot_z.setter
    def robot_z(self, v): self.physics.robot_z = v
    @property
    def robot_rotation(self): return self.physics.robot_rotation
    @robot_rotation.setter
    def robot_rotation(self, v): self.physics.robot_rotation = v
    @property
    def robot_y_offset(self): return self.physics.robot_y_offset
    @property
    def servo_angles(self): return self.kinematics.servo_angles
    @property
    def filtered_signal(self): return self.state.filtered_signal
    @property
    def raw_signal(self): return self.state.raw_signal
    @property
    def distance(self): return self.state.filtered_signal # Proxy for 3D widget
    @property
    def femur_len(self): return self.kinematics.femur_len
    @femur_len.setter
    def femur_len(self, v): self.kinematics.femur_len = v
    @property
    def tibia_len(self): return self.kinematics.tibia_len
    @tibia_len.setter
    def tibia_len(self, v): self.kinematics.tibia_len = v
    @property
    def highlighted_obs_id(self): return self.physics.highlighted_obs_id
    @property
    def custom_obstacles(self): return self.physics.custom_obstacles
    @property
    def quantized_event(self): return self.state.quantized_event

    def stop(self):
        self.is_running = False
        self.worker_thread.quit()
        self.worker_thread.wait()

    def reset(self):
        self.physics = PhysicsEngine()
        self.kinematics = RobotKinematics()
        self.state = RobotStateMachine()
        self.time_elapsed = 0.0
        self.planned_path = []
        self.nav_target = None
        self.transition_count = 0
        self.just_transitioned = False
        self.occupancy_grid = OccupancyGrid() # Reset map too


