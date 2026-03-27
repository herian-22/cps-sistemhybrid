from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QSlider, QPushButton, QFrame, QGridLayout,
                             QCheckBox, QScrollArea, QSizePolicy, QTabWidget,
                             QSpinBox, QListWidget, QListWidgetItem)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from core.logic import HybridSystemLogic, SystemMode, ControlMode
from ui.widgets.robot_3d import Robot3DWidget
from ui.widgets.analytics import (StateDiagramWidget, RealTimePlotWidget, SignalPanel, 
                                 SignalAnalysisWidget, ModeTransitionWidget, 
                                 TrajectoryWidget, ServoResponseWidget)
from ui.widgets.gauges import ServoWidget, DistanceWidget

# ── Color palette ─────────────────────────────────────────────────────────────
C = dict(
    bg="#0f172a", surface="#1e293b", border="#334155",
    text="#e2e8f0", muted="#64748b",
    blue="#3b82f6", green="#10b981", red="#ef4444",
    amber="#f59e0b", violet="#7c3aed",
)

APP_STYLE = """
QMainWindow, QWidget  { background-color: %(bg)s; color: %(text)s; font-family: 'Segoe UI', sans-serif; }
QFrame#Navbar         { background-color: %(surface)s; border-bottom: 1px solid %(border)s; }
QFrame#SideOuter      { background-color: %(surface)s; border-right: 1px solid %(border)s; }
QFrame#SideSection    { background-color: %(bg)s; border: 1px solid %(border)s; border-radius: 8px; }
QScrollArea           { border: none; background: transparent; }
QScrollBar:vertical   { background: %(bg)s; width: 6px; border-radius: 3px; }
QScrollBar::handle:vertical { background: %(border)s; border-radius: 3px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QLabel                { color: %(text)s; }
QLabel#SecTitle       { color: %(muted)s; font-size: 10px; font-weight: bold; letter-spacing: 1px; }
QPushButton           { background: %(blue)s; color: white; border: none; border-radius: 6px;
                        padding: 8px 12px; font-weight: bold; font-size: 12px; }
QPushButton:hover     { background: #2563eb; }
QPushButton:pressed   { background: #1d4ed8; }
QPushButton#Sec       { background: #334155; color: %(text)s; }
QPushButton#Sec:hover { background: #475569; }
QPushButton#Zeno      { background: %(violet)s; }
QPushButton#Zeno:hover{ background: #6d28d9; }
QPushButton#ZenoOn    { background: %(red)s; border: 2px solid #fca5a5; }
QPushButton#Auto      { background: %(amber)s; color: #000; }
QPushButton#Auto:hover{ background: #d97706; }
QPushButton#NavBtn    { background: transparent; color: %(muted)s; border-radius: 4px;
                        padding: 5px 12px; font-size: 12px; }
QPushButton#NavBtn:hover { color: %(text)s; background: #334155; }
QSlider::groove:horizontal { border: 1px solid %(border)s; height: 6px; background: %(bg)s; border-radius: 3px; }
QSlider::handle:horizontal { background: %(blue)s; width: 16px; height: 16px; margin: -5px 0; border-radius: 8px; }
QSlider::sub-page:horizontal { background: %(blue)s; border-radius: 3px; }
QCheckBox             { color: %(muted)s; spacing: 5px; }
QCheckBox::indicator  { width: 15px; height: 15px; border: 1px solid %(border)s; border-radius: 3px; }
QCheckBox::indicator:checked { background: %(blue)s; border-color: %(blue)s; }
QTabWidget::pane      { border-top: 2px solid %(border)s; background: %(bg)s; }
QTabBar::tab          { background: %(surface)s; color: %(muted)s; padding: 10px 20px;
                        border: 1px solid %(border)s; border-bottom: none;
                        border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 2px; }
QTabBar::tab:selected { background: %(blue)s; color: white; border-color: %(blue)s; font-weight: bold; }
QSpinBox              { background: %(bg)s; color: %(text)s; border: 1px solid %(border)s; padding: 4px; border-radius: 4px; }
QListWidget           { background: %(bg)s; color: %(text)s; border: 1px solid %(border)s; border-radius: 4px; padding: 4px; }
QListWidget::item     { padding: 8px; border-bottom: 1px solid %(surface)s; }
QListWidget::item:selected { background: %(blue)s; color: white; border-radius: 4px; }
""" % C


def _sec(title, widgets):
    """Build a labelled section card."""
    f = QFrame(); f.setObjectName("SideSection")
    v = QVBoxLayout(f); v.setContentsMargins(10, 8, 10, 8); v.setSpacing(5)
    lb = QLabel(title.upper()); lb.setObjectName("SecTitle"); v.addWidget(lb)
    for w in widgets:
        if isinstance(w, str):
            lbl = QLabel(w); lbl.setStyleSheet(f"color:{C['muted']}; font-size:10px;")
            lbl.setWordWrap(True); v.addWidget(lbl)
        else:
            v.addWidget(w)
    return f


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CPS Hybrid System Simulator")
        self.setMinimumSize(1050, 700)
        self.resize(1380, 870)
        self.setStyleSheet(APP_STYLE)
        self.logic = HybridSystemLogic()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.timer.start(50)
        self._init_ui()

    # ── Build UI ──────────────────────────────────────────────────────────────
    def _init_ui(self):
        root = QWidget(); self.setCentralWidget(root)
        vlay = QVBoxLayout(root)
        vlay.setContentsMargins(0, 0, 0, 0); vlay.setSpacing(0)
        vlay.addWidget(self._navbar())

        self.tabs = QTabWidget()
        vlay.addWidget(self.tabs, 1)

        # Tab 1: Live Simulation
        self.live_tab = QWidget()
        body = QHBoxLayout(self.live_tab); body.setContentsMargins(0,0,0,0); body.setSpacing(0)
        body.addWidget(self._sidebar())
        body.addLayout(self._viewport(), 1)
        self.tabs.addTab(self.live_tab, "Live Simulation")

        # Tab 2: Analytics Dashboard (Deep Analysis)
        self.analytics_tab = QWidget()
        self._build_analytics_tab(self.analytics_tab)
        self.tabs.addTab(self.analytics_tab, "Analytics Dashboard")

        # Tab 3: Global Configuration
        self.config_tab = QWidget()
        self._build_config_tab(self.config_tab)
        self.tabs.addTab(self.config_tab, "Global Configuration")

    def _navbar(self):
        bar = QFrame(); bar.setObjectName("Navbar"); bar.setFixedHeight(50)
        lay = QHBoxLayout(bar); lay.setContentsMargins(16, 0, 16, 0); lay.setSpacing(8)
        logo = QLabel("⬡ CPS LAB")
        logo.setStyleSheet(f"color:{C['blue']};font-size:16px;font-weight:bold;")
        lay.addWidget(logo)
        sep = QFrame(); sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet(f"color:{C['border']};"); lay.addWidget(sep)
        for name in ("Dashboard","3D Simulation","Analytics"):
            btn = QPushButton(name); btn.setObjectName("NavBtn")
            lay.addWidget(btn)
        lay.addStretch()
        self.mode_badge = QLabel("● WALKING")
        self.mode_badge.setStyleSheet(f"color:{C['green']};font-weight:bold;font-size:14px;")
        lay.addWidget(self.mode_badge)
        self.trans_label = QLabel("  Transitions: 0")
        self.trans_label.setStyleSheet(f"color:{C['muted']};font-size:12px;")
        lay.addWidget(self.trans_label)
        return bar

    def _sidebar(self):
        outer = QFrame(); outer.setObjectName("SideOuter")
        outer.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        outer.setFixedWidth(270)
        vlay = QVBoxLayout(outer); vlay.setContentsMargins(0,0,0,0); vlay.setSpacing(0)

        # Scrollable interior
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        inner = QWidget()
        ilay  = QVBoxLayout(inner); ilay.setContentsMargins(10,10,10,10); ilay.setSpacing(8)
        ilay.setAlignment(Qt.AlignTop)

        # ── Simulation ────────────────────────────────────────────────────────
        self.btn_run  = QPushButton("▶  START"); self.btn_run.clicked.connect(self.toggle_run)
        self.btn_mode = QPushButton("🤖  AUTO MODE"); self.btn_mode.setObjectName("Sec")
        self.btn_mode.setToolTip("AUTO: robot advances automatically and evades")
        self.btn_mode.clicked.connect(self.toggle_control)
        btn_reset = QPushButton("↺  RESET"); btn_reset.setObjectName("Sec")
        btn_reset.clicked.connect(self.reset_sim)
        self.btn_zeno = QPushButton("⚡  ZENO MODE"); self.btn_zeno.setObjectName("Zeno")
        self.btn_zeno.clicked.connect(self.toggle_zeno)
        ilay.addWidget(_sec("Simulation", [self.btn_run, self.btn_mode, btn_reset,
                                           self.btn_zeno,
                                           "Zeno: collapses hysteresis + injects noise."]))

        # ── Sensor ────────────────────────────────────────────────────────────
        self.dist_view   = DistanceWidget()
        self.dist_slider = QSlider(Qt.Horizontal)
        self.dist_slider.setRange(5, 100); self.dist_slider.setValue(100)
        self.dist_slider.valueChanged.connect(self.on_dist)
        self.sensor_sec = _sec("Sensor Distance (Manual)", [self.dist_view, self.dist_slider])
        ilay.addWidget(self.sensor_sec)

        # ── Camera ────────────────────────────────────────────────────────────
        btn_cam = QPushButton("↺  Reset Camera"); btn_cam.setObjectName("Sec")
        btn_cam.clicked.connect(self.reset_camera)
        ilay.addWidget(_sec("Camera", ["🖱 Drag: Orbit  |  Scroll: Zoom\n↕ Fully unclamped vertical range", btn_cam]))

        # ── View options ──────────────────────────────────────────────────────
        self.chk_grid   = QCheckBox("Floor Grid"); self.chk_grid.setChecked(True)
        self.chk_axes   = QCheckBox("Axis Gizmo"); self.chk_axes.setChecked(True)
        self.chk_labels = QCheckBox("Labels");     self.chk_labels.setChecked(True)
        self.chk_grid.stateChanged.connect(lambda s: (setattr(self.robot_3d,'show_grid',s==2), self.robot_3d.update()))
        self.chk_axes.stateChanged.connect(lambda s: (setattr(self.robot_3d,'show_axes',s==2), self.robot_3d.update()))
        self.chk_labels.stateChanged.connect(lambda s: (setattr(self.robot_3d,'show_labels',s==2), self.robot_3d.update()))
        ilay.addWidget(_sec("View", [self.chk_grid, self.chk_axes, self.chk_labels]))

        # ── Servo gauges ──────────────────────────────────────────────────────
        self.gauges = []
        gg = QWidget(); gl = QGridLayout(gg); gl.setContentsMargins(0,0,0,0); gl.setSpacing(4)
        for i in range(4):
            g = ServoWidget(f"S{i+1}"); self.gauges.append(g); g.setFixedHeight(105)
            gl.addWidget(g, i//2, i%2)
        ilay.addWidget(_sec("Servo Gauges", [gg]))

        scroll.setWidget(inner); vlay.addWidget(scroll)
        return outer

    def _viewport(self):
        lay = QVBoxLayout(); lay.setContentsMargins(0,0,0,0); lay.setSpacing(0)

        # Container for 3D View and HUD Overlays (Z-stacking)
        self.view_container = QWidget()
        v_glay = QGridLayout(self.view_container)
        v_glay.setContentsMargins(0,0,0,0)

        self.robot_3d = Robot3DWidget()
        self.robot_3d.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        v_glay.addWidget(self.robot_3d, 0, 0)

        # HUD Overlay (Digital Twin Status)
        self.hud = QLabel("MCU: ACTIVE\nBATT: 98.0%\nDIST: 100 CM\nMODE: WALKING")
        self.hud.setStyleSheet("color: #10b981; background: rgba(15, 23, 42, 180); "
                               "padding: 15px; border-radius: 10px; border: 1px solid #334155; "
                               "font-family: 'Consolas', 'Courier New', monospace; font-size: 13px; margin: 15px;")
        self.hud.setAttribute(Qt.WA_TransparentForMouseEvents)
        v_glay.addWidget(self.hud, 0, 0, Qt.AlignTop | Qt.AlignRight)

        lay.addWidget(self.view_container, 3)

        # Analytics strip — fixed height so it doesn't squash the 3D view
        strip = QFrame()
        strip.setFixedHeight(200)
        strip.setStyleSheet(f"background:{C['surface']};border-top:1px solid {C['border']};")
        slay = QHBoxLayout(strip); slay.setContentsMargins(10,6,10,6); slay.setSpacing(10)

        self.state_diag = StateDiagramWidget(); self.state_diag.setFixedWidth(280)
        slay.addWidget(self.state_diag)

        vsep = QFrame(); vsep.setFrameShape(QFrame.VLine)
        vsep.setStyleSheet(f"color:{C['border']}"); slay.addWidget(vsep)

        pcol = QVBoxLayout()
        servo_lbl = QLabel("SERVO ANGLES"); servo_lbl.setObjectName("SecTitle")
        pcol.addWidget(servo_lbl)
        self.plot = RealTimePlotWidget()
        pcol.addWidget(self.plot)
        slay.addLayout(pcol, 1)

        vsep2 = QFrame(); vsep2.setFrameShape(QFrame.VLine)
        vsep2.setStyleSheet(f"color:{C['border']}"); slay.addWidget(vsep2)

        scol = QVBoxLayout()
        sig_lbl = QLabel("SIGNAL: CONTINUOUS → DISCRETE"); sig_lbl.setObjectName("SecTitle")
        scol.addWidget(sig_lbl)
        self.sig_panel = SignalPanel()
        scol.addWidget(self.sig_panel)
        slay.addLayout(scol, 1)

        lay.addWidget(strip)
        return lay

    def _build_analytics_tab(self, parent):
        play = QVBoxLayout(parent); play.setContentsMargins(20,20,20,20); play.setSpacing(20)
        
        header = QLabel("DEEP ANALYTICS DASHBOARD")
        header.setStyleSheet(f"color:{C['blue']}; font-size:18px; font-weight:bold; letter-spacing:1px;")
        play.addWidget(header)

        grid = QGridLayout()
        grid.setSpacing(20)
        
        # 1. Signal Filter Analysis
        self.ana_signal = SignalAnalysisWidget(
            "Grafik Perbandingan Sinyal (Filter Analysis)",
            "Tunjukkan bagaimana filter IIR berhasil meredam noise sensor sehingga sistem tidak melakukan transisi 'palsu'."
        )
        grid.addWidget(self.ana_signal, 0, 0)
        
        # 2. Mode Transition
        self.ana_mode = ModeTransitionWidget(
            "Grafik Transisi Mode (Hybrid Automata)",
            "Tunjukkan titik waktu tepat saat robot mendeteksi rintangan (distance < 20) dan mengubah perilakunya."
        )
        grid.addWidget(self.ana_mode, 0, 1)
        
        # 3. Trajectory
        self.ana_traj = TrajectoryWidget(
            "Grafik Koordinat Robot (Trajectory)",
            "Jika garisnya lurus, robot berjalan konstan. Jika ada lekukan atau berhenti, itu menunjukkan robot sedang dalam mode EVASIVE atau terkena efek pantulan (bounce)."
        )
        grid.addWidget(self.ana_traj, 1, 0)
        
        # 4. Actuator Response
        self.ana_actuator = ServoResponseWidget(
            "Analisis Sudut Servo (Actuator Response)",
            "Lihat pola gelombang sinus saat WALKING dan bagaimana polanya berubah drastis menjadi garis statis saat masuk ke mode EVASIVE."
        )
        grid.addWidget(self.ana_actuator, 1, 1)
        
        play.addLayout(grid)
        play.addStretch()

    def _build_config_tab(self, parent):
        play = QHBoxLayout(parent); play.setContentsMargins(30,30,30,30); play.setSpacing(30)

        # Left Column: Kinematics & Gait
        left_col = QVBoxLayout(); left_col.setAlignment(Qt.AlignTop)

        # Kinematics
        kf = QFrame(); kl = QVBoxLayout(kf); kf.setObjectName("SideSection")
        kl.addWidget(QLabel("INVERSE KINEMATICS (IK) - LEG PROPORTIONS"))

        self.chk_ik = QCheckBox("Enable Inverse Kinematics Mechanism")
        self.chk_ik.setChecked(True)
        self.chk_ik.toggled.connect(lambda c: setattr(self.logic, 'use_ik', c))
        kl.addWidget(self.chk_ik)

        self.lbl_femur=QLabel("Femur Length: 50.0 cm"); kl.addWidget(self.lbl_femur)
        self.sl_femur = QSlider(Qt.Horizontal); self.sl_femur.setRange(20, 100); self.sl_femur.setValue(50)
        self.sl_femur.valueChanged.connect(lambda v: (setattr(self.logic,'femur_len',float(v)), self.lbl_femur.setText(f"Femur Length: {v}.0 cm")))
        kl.addWidget(self.sl_femur)

        self.lbl_tibia=QLabel("Tibia Length: 50.0 cm"); kl.addWidget(self.lbl_tibia)
        self.sl_tibia = QSlider(Qt.Horizontal); self.sl_tibia.setRange(20, 100); self.sl_tibia.setValue(50)
        self.sl_tibia.valueChanged.connect(lambda v: (setattr(self.logic,'tibia_len',float(v)), self.lbl_tibia.setText(f"Tibia Length: {v}.0 cm")))
        kl.addWidget(self.sl_tibia)
        left_col.addWidget(kf)

        # Gait Parameters
        gf = QFrame(); gl = QVBoxLayout(gf); gf.setObjectName("SideSection")
        gl.addWidget(QLabel("CONTINUOUS GAIT DYNAMICS"))
        self.lbl_amp=QLabel("Walking Amplitude: 30°"); gl.addWidget(self.lbl_amp)
        self.sl_amp = QSlider(Qt.Horizontal); self.sl_amp.setRange(10, 60); self.sl_amp.setValue(30)
        self.sl_amp.valueChanged.connect(lambda v: (setattr(self.logic,'walking_amplitude',float(v)), self.lbl_amp.setText(f"Walking Amplitude: {v}°")))
        gl.addWidget(self.sl_amp)

        self.lbl_spd=QLabel("Walking Speed: 5.0 rad/s"); gl.addWidget(self.lbl_spd)
        self.sl_spd = QSlider(Qt.Horizontal); self.sl_spd.setRange(1, 15); self.sl_spd.setValue(5)
        self.sl_spd.valueChanged.connect(lambda v: (setattr(self.logic,'walking_speed',float(v)), self.lbl_spd.setText(f"Walking Speed: {v}.0 rad/s")))
        gl.addWidget(self.sl_spd)
        left_col.addWidget(gf)
        left_col.addStretch()

        # Right Column: Obstacle Manager
        right_col = QVBoxLayout(); right_col.setAlignment(Qt.AlignTop)
        of = QFrame(); ol = QVBoxLayout(of); of.setObjectName("SideSection")
        ol.addWidget(QLabel("CUSTOM OBSTACLE MANAGER"))
        ol.addWidget(QLabel("Add static obstacles. Evasion triggers if distance to robot front ≤ 25cm."))

        form = QGridLayout(); form.setContentsMargins(0,10,0,10)
        form.addWidget(QLabel("X (lateral):"), 0,0); self.spin_x = QSpinBox(); self.spin_x.setRange(-500,500); self.spin_x.setValue(0); form.addWidget(self.spin_x, 0,1)
        form.addWidget(QLabel("Z (depth):"), 0,2);   self.spin_z = QSpinBox(); self.spin_z.setRange(0,3000); self.spin_z.setValue(300); form.addWidget(self.spin_z, 0,3)
        form.addWidget(QLabel("Width:"), 1,0);       self.spin_w = QSpinBox(); self.spin_w.setRange(10,500); self.spin_w.setValue(60); form.addWidget(self.spin_w, 1,1)
        form.addWidget(QLabel("Height:"), 1,2);      self.spin_h = QSpinBox(); self.spin_h.setRange(10,500); self.spin_h.setValue(90); form.addWidget(self.spin_h, 1,3)
        form.addWidget(QLabel("Depth:"), 2,0);       self.spin_d = QSpinBox(); self.spin_d.setRange(10,500); self.spin_d.setValue(60); form.addWidget(self.spin_d, 2,1)
        ol.addLayout(form)

        btn_add = QPushButton("➕ ADD OBSTACLE"); btn_add.clicked.connect(self.add_custom_obs)
        ol.addWidget(btn_add)

        self.obs_list = QListWidget(); self.obs_list.setFixedHeight(180)
        ol.addWidget(self.obs_list)
        self.refresh_obs_list()

        btn_del = QPushButton("❌ REMOVE SELECTED")
        btn_del.setObjectName("Sec"); btn_del.clicked.connect(self.del_custom_obs)
        ol.addWidget(btn_del)

        right_col.addWidget(of)
        right_col.addStretch()

        play.addLayout(left_col, 1)
        play.addLayout(right_col, 1)

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def add_custom_obs(self):
        x, z = self.spin_x.value(), self.spin_z.value()
        w, h = self.spin_w.value(), self.spin_h.value()
        d = self.spin_d.value()
        self.logic.add_custom_obstacle(x, z, w, h, d)
        self.refresh_obs_list()

    def del_custom_obs(self):
        row = self.obs_list.currentRow()
        if row >= 0:
            self.logic.remove_custom_obstacle(row)
            self.refresh_obs_list()

    def refresh_obs_list(self):
        self.obs_list.clear()
        for i, o in enumerate(self.logic.custom_obstacles):
            self.obs_list.addItem(f"[{o.id}] X:{int(o.x)} Z:{int(o.z)} | W:{o.width} H:{o.height} D:{o.depth}")

    def on_dist(self, v):
        self.dist_view.setDistance(float(v))

    def toggle_run(self):
        self.logic.is_running = not self.logic.is_running
        self.btn_run.setText("⏸  PAUSE" if self.logic.is_running else "▶  RESUME")

    def toggle_control(self):
        if self.logic.control == ControlMode.MANUAL:
            self.logic.set_control(ControlMode.AUTO)
            self.btn_mode.setText("🕹  MANUAL MODE"); self.btn_mode.setObjectName("Auto")
            self.sensor_sec.setEnabled(False)
        else:
            self.logic.set_control(ControlMode.MANUAL)
            self.btn_mode.setText("🤖  AUTO MODE"); self.btn_mode.setObjectName("Sec")
            self.sensor_sec.setEnabled(True)
        self.btn_mode.setStyle(self.btn_mode.style())

    def toggle_zeno(self):
        self.logic.zeno_mode = not self.logic.zeno_mode
        if self.logic.zeno_mode:
            self.btn_zeno.setObjectName("ZenoOn")
            self.btn_zeno.setText("⚠  ZENO ACTIVE")
            self.dist_slider.setValue(25)
        else:
            self.btn_zeno.setObjectName("Zeno")
            self.btn_zeno.setText("⚡  ZENO MODE")
        self.btn_zeno.setStyle(self.btn_zeno.style())

    def reset_sim(self):
        self.logic.reset()
        self.logic.is_running = False
        self.dist_slider.setValue(100)
        self.btn_run.setText("▶  START")
        self.refresh_obs_list()

    def reset_camera(self):
        self.robot_3d.rot_x = 0.5
        self.robot_3d.rot_y = 0.15
        self.robot_3d.zoom  = 800
        self.robot_3d.update()

    # ── Update loop ───────────────────────────────────────────────────────────
    def update_simulation(self):
        if self.logic.is_running:
            self.logic.update()

        self.robot_3d.setParams(self.logic)
        self.state_diag.setMode(self.logic.mode, self.logic.just_transitioned)
        self.plot.setData(self.logic.history_servos)
        self.sig_panel.setData(
            self.logic.history_raw,
            self.logic.history_filtered,
            self.logic.quantized_event
        )
        
        # New Analytics Dashboard update
        self.ana_signal.setData(self.logic.history_raw, self.logic.history_filtered)
        self.ana_mode.setData(self.logic.history_mode)
        self.ana_traj.setData(self.logic.history_z)
        self.ana_actuator.setData(self.logic.history_servos[:2]) # Only S1 and S2
        
        # HUD Overlay Update
        hud_col = "#10b981" if self.logic.mode == SystemMode.WALKING else "#ef4444"
        if self.logic.mcu_status != "ACTIVE": hud_col = "#f59e0b"
        self.hud.setText(f"MCU: {self.logic.mcu_status}\n"
                         f"BATT: {self.logic.battery_level:.1f}%\n"
                         f"DIST: {self.logic.distance:.1f} CM\n"
                         f"MODE: {self.logic.mode.value}")
        self.hud.setStyleSheet(f"color: {hud_col}; background: rgba(15, 23, 42, 180); "
                               "padding: 15px; border-radius: 10px; border: 1px solid #334155; "
                               "font-family: 'Consolas', monospace; font-size: 13px; margin: 15px;")

        for i in range(4):
            self.gauges[i].setAngle(self.logic.servo_angles[i])

        if self.logic.is_running:
            self.dist_view.setDistance(self.logic.distance)

        # Navbar badges
        walking = (self.logic.mode == SystemMode.WALKING)
        col = C['green'] if walking else C['red']
        self.mode_badge.setText(f"● {self.logic.mode.value}")
        self.mode_badge.setStyleSheet(f"color:{col};font-weight:bold;font-size:14px;")
        zt = " ⚡" if self.logic.zeno_mode else ""
        at = " [AUTO]" if self.logic.control == ControlMode.AUTO else ""
        self.trans_label.setText(f"  Transitions: {self.logic.transition_count}{zt}{at}")
