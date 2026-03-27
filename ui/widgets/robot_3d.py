import math
import time
import numpy as np
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QPoint, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPolygonF
from ui.utils.projection import project

FLOOR_Y   = -120  # ground plane Y coordinate


class Robot3DWidget(QWidget):
    """
    3D visualization with:
    - Painter's Algorithm depth sorting
    - Floor-snapped obstacles (rendered from logic.obstacles list)
    - IK legs, shaded body, sensor beam, compass gizmo
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.angles    = [90.0]*8
        self.distance  = 100.0
        self.femur_len = 50.0
        self.tibia_len = 50.0
        self.robot_x   = 0.0
        self.robot_z   = 0.0
        self.robot_rotation = 0.0
        self.robot_y_offset = 0.0
        self.use_ik    = True
        self.custom_obstacles = []
        self.planned_path = []
        self.highlighted_obs_id = None

        self.rot_x = 0.5
        self.rot_y = 0.15
        self.zoom  = 800
        self.focal_length = 500
        self.last_mouse_pos = QPoint()

        self.show_grid   = True
        self.show_axes   = True
        self.show_labels = True

    def setParams(self, logic):
        self.angles    = logic.servo_angles
        self.distance      = logic.distance
        self.femur_len     = logic.femur_len
        self.tibia_len     = logic.tibia_len
        self.robot_x       = logic.robot_x
        self.robot_z       = logic.robot_z
        self.robot_rotation= logic.robot_rotation
        self.robot_y_offset= logic.robot_y_offset
        self.use_ik        = logic.use_ik
        self.custom_obstacles   = logic.custom_obstacles
        self.occupancy_grid     = logic.occupancy_grid.grid
        self.planned_path       = logic.planned_path
        self.highlighted_obs_id = logic.highlighted_obs_id
        self.update()

    def p3(self, x, y, z):
        return project(x, y, z, self.width(), self.height(),
                       self.rot_x, self.rot_y, self.focal_length, self.zoom)

    def world_to_local(self, wx, wz):
        dx = wx - self.robot_x
        dz = wz - self.robot_z
        rad = math.radians(-self.robot_rotation)
        lx = dx * math.cos(rad) - dz * math.sin(rad)
        lz = dx * math.sin(rad) + dz * math.cos(rad)
        return lx, lz

    def mousePressEvent(self, e):
        self.last_mouse_pos = e.position().toPoint()

    def mouseMoveEvent(self, e):
        if e.buttons() & Qt.LeftButton:
            curr = e.position().toPoint()
            diff = curr - self.last_mouse_pos
            self.rot_y -= diff.x() * 0.01
            self.rot_x += diff.y() * 0.01
            self.last_mouse_pos = curr
            self.update()

    def wheelEvent(self, e):
        self.zoom = max(150, min(2500, self.zoom - e.angleDelta().y()*0.5))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        t = time.time()

        if self.show_grid:
            self._draw_floor_grid(painter)
            self._draw_path(painter)

        dl = []
        try:
            painter.setRenderHint(QPainter.Antialiasing)
            t = time.time()

            if self.show_grid:
                self._draw_floor_grid(painter)
                self._draw_path(painter)

            dl = []

            # ── Unmapped Physical Obstacles (Holographic Blue) ────────────────────
            c_holo = (QColor(14,165,233,60), QColor(2,132,199,50), QColor(3,105,161,40)) 
            c_high = (QColor(244,63,94,220),  QColor(225,29,72,210),  QColor(190,18,60,200)) 
            for obs in self.custom_obstacles:
                c = c_high if getattr(self, 'highlighted_obs_id', None) == obs.id else c_holo
                if abs(obs.x - self.robot_x) > 4000 or abs(obs.z - self.robot_z) > 4000: continue
                lx, lz = self.world_to_local(obs.x, obs.z)
                if lz < -500: continue
                dl.append((float(lz),
                           (lambda _obs, _c: lambda p:
                            self._draw_box(p, _obs.x, FLOOR_Y, _obs.z,
                                           _obs.width, _obs.height, _obs.depth,
                                           _c[0], _c[1], _c[2], glow=(_obs.id==getattr(self, 'highlighted_obs_id', None))))(obs, c)))

            # ── Map obstacles (Occupancy Grid) (Solid Concrete) ───────────────────
            from core.pathfinding import CELL_SIZE, OFFSET
            cc = (QColor(100,116,139,220), QColor(71,85,105,210), QColor(51,65,85,200)) # Concrete gray
            
            if hasattr(self, 'occupancy_grid') and self.occupancy_grid is not None:
                # Find all cells with occupancy > 0.9
                # If it's a grid object, use .grid
                grid_data = getattr(self.occupancy_grid, 'grid', self.occupancy_grid)
                off = getattr(self.occupancy_grid, 'OFFSET', OFFSET)
                res = getattr(self.occupancy_grid, 'RESOLUTION', CELL_SIZE)
                
                if isinstance(grid_data, np.ndarray):
                    gz_indices, gx_indices = np.where(grid_data > 0.9)
                    
                    for gz, gx in zip(gz_indices, gx_indices):
                        wx = (gx - off) * res
                        wz = (gz - off) * res
                        
                        if abs(wx - self.robot_x) > 1500 or abs(wz - self.robot_z) > 1500: continue
                        
                        lx, lz = self.world_to_local(wx, wz)
                        if lz < -500: continue
                        
                        dl.append((float(lz),
                                (lambda _wx, _wz, _c: lambda p:
                                    self._draw_box(p, _wx, FLOOR_Y, _wz,
                                                res, 80, res,
                                                _c[0], _c[1], _c[2], glow=False))(wx, wz, cc)))

            if self.planned_path:
                dl.append((0.0, self._draw_planned_path))

            dl.append((1.0,  self._draw_ground_shadow))
            dl.append((0.0,  self._draw_robot_body))
            dl.append((-5.0, lambda p: self._draw_ultrasonic_waves(p, 160, t)))

            bw, bl = 100, 160
            mounts = [(-bw/2,0,-bl/2),(bw/2,0,-bl/2),(-bw/2,0,bl/2),(bw/2,0,bl/2)]
            colors = ["#ef4444","#3b82f6","#10b981","#f59e0b"]
            for i,(lx,ly,lz) in enumerate(mounts):
                q1 = math.radians(self.angles[i*2] - 90)
                q2 = math.radians(self.angles[i*2+1] - 90)
                
                if self.use_ik:
                    # Base sort depth approximation on leg joints
                    kx,ky,kz = lx, ly+self.femur_len*math.sin(q1)+self.robot_y_offset, lz+self.femur_len*math.cos(q1)
                    fx,fy,fz = kx, ky-self.robot_y_offset+self.tibia_len*math.sin(q1+q2), kz+self.tibia_len*math.cos(q1+q2)
                    dl.append(((lz+kz+fz)/3,
                               (lambda _i,_lx,_ly,_lz,_q1,_q2,_c,_ro:
                                lambda p: self._draw_ik_leg(p,_i,_lx,_ly,_lz,_q1,_q2,_c,_ro)
                                )(i,lx,ly,lz,q1,q2,colors[i], self.robot_y_offset)))
                else:
                    # Rigid straight leg direct to floor if IK disabled
                    fx = lx
                    fy = ly - (self.femur_len + self.tibia_len) * 0.8
                    fz = -(self.femur_len + self.tibia_len) * math.cos(q1)
                    dl.append(((lz+fz)/2,
                               (lambda _i,_lx,_ly,_lz,_fx,_fy,_fz,_c,_ro:
                                lambda p: self._draw_rigid_leg(p,_i,_lx,_ly,_lz,_fx,_fy,_fz,_c,_ro)
                                )(i,lx,ly,lz,fx,fy,fz,colors[i], self.robot_y_offset)))

            dl.sort(key=lambda x: x[0], reverse=True)
            for _, fn in dl:
                fn(painter)

            if self.show_axes:
                self._draw_coordinate_axes(painter)
                self._draw_compass(painter)
        finally:
            painter.end()

    def _draw_floor_grid(self, painter):
        gc = QColor(71,85,105, 50)
        
        # Draw world-aligned grid across a local viewing box
        view_radius = 2500
        grid_size = 150
        
        start_x = int((self.robot_x - view_radius) / grid_size) * grid_size
        end_x   = int((self.robot_x + view_radius) / grid_size) * grid_size
        start_z = int((self.robot_z - view_radius) / grid_size) * grid_size
        end_z   = int((self.robot_z + view_radius) / grid_size) * grid_size

        painter.setPen(QPen(gc, 1))
        # Z-lines
        for x in range(start_x, end_x + 1, grid_size):
            p1 = self.p3(*self.world_to_local(x, start_z)[:1], FLOOR_Y, self.world_to_local(x, start_z)[1])
            p2 = self.p3(*self.world_to_local(x, end_z)[:1], FLOOR_Y, self.world_to_local(x, end_z)[1])
            if p1 and p2: painter.drawLine(p1, p2)
            
        # X-lines
        for z in range(start_z, end_z + 1, grid_size):
            p1 = self.p3(*self.world_to_local(start_x, z)[:1], FLOOR_Y, self.world_to_local(start_x, z)[1])
            p2 = self.p3(*self.world_to_local(end_x, z)[:1], FLOOR_Y, self.world_to_local(end_x, z)[1])
            if p1 and p2: painter.drawLine(p1, p2)

    def _draw_planned_path(self, painter):
        painter.setPen(QPen(QColor(16, 185, 129, 200), 4, Qt.DashLine))
        pts = []
        for (wx, wz) in self.planned_path:
            lx, lz = self.world_to_local(wx, wz)
            pts.append(self.p3(lx, FLOOR_Y+2, lz))
        if pts:
            # Connect current pos to first path node
            p0 = self.p3(0, FLOOR_Y+2, 0)
            painter.drawLine(p0, pts[0])
            painter.drawPolyline(QPolygonF(pts))

    def _draw_path(self, painter):
        pass # Disabling static path polygon in true 2D mode

    def _draw_ground_shadow(self, painter):
        sp = self.p3(0,FLOOR_Y,0)
        painter.setPen(Qt.NoPen); painter.setBrush(QBrush(QColor(0,0,0,55)))
        painter.drawEllipse(sp, 95, 22)

    def _draw_box(self, painter, wx, base_y, wz, bw, bh, bd,
                   front_col, side_col, top_col, glow=False):
        hw, hd  = bw/2, bd/2
        top_y   = base_y + bh
        
        # 8 corners in World Space
        w_c = [
            (wx-hw,base_y,wz-hd),(wx+hw,base_y,wz-hd),(wx+hw,top_y,wz-hd),(wx-hw,top_y,wz-hd),
            (wx-hw,base_y,wz+hd),(wx+hw,base_y,wz+hd),(wx+hw,top_y,wz+hd),(wx-hw,top_y,wz+hd),
        ]
        
        # Convert to Local Space
        l_c = []
        for (cx, cy, cz) in w_c:
            lx, lz = self.world_to_local(cx, cz)
            l_c.append((lx, cy, lz))
            
        p = [self.p3(*v) for v in l_c]
        
        def face(idx, col):
            painter.setBrush(QBrush(col)); painter.setPen(QPen(col.darker(140),1))
            painter.drawPolygon(QPolygonF([p[i] for i in idx]))

        if glow:
            painter.setPen(QPen(QColor(244,63,94,150), 3))
            painter.setBrush(Qt.NoBrush)
            painter.drawPolygon(QPolygonF([p[i] for i in [0,1,2,3]]))
            painter.drawPolygon(QPolygonF([p[i] for i in [3,2,6,7]]))

        face([7,6,5,4], side_col.darker(115))
        face([0,4,7,3], side_col); face([1,5,6,2], side_col)
        face([0,1,5,4], top_col.darker(110)); face([3,2,6,7], top_col)
        face([0,1,2,3], front_col)

    def _draw_robot_body(self, painter):
        bw,bh,bl = 100,30,160
        y_off = self.robot_y_offset
        c = [(-bw/2,y_off-bh/2,-bl/2),(bw/2,y_off-bh/2,-bl/2),(bw/2,y_off+bh/2,-bl/2),(-bw/2,y_off+bh/2,-bl/2),
             (-bw/2,y_off-bh/2, bl/2),(bw/2,y_off-bh/2, bl/2),(bw/2,y_off+bh/2, bl/2),(-bw/2,y_off+bh/2, bl/2)]
        p = [self.p3(*v) for v in c]
        def bf(idx,col):
            painter.setBrush(QBrush(col)); painter.setPen(QPen(QColor("#2d3f55"),1))
            painter.drawPolygon(QPolygonF([p[i] for i in idx]))
        bf([0,4,7,3],QColor(28,38,55,235)); bf([1,5,6,2],QColor(28,38,55,235))
        bf([0,1,5,4],QColor(45,58,80,240)); bf([3,2,6,7],QColor(45,58,80,240))
        bf([4,5,6,7],QColor(60,75,100,245)); bf([0,1,2,3],QColor(95,115,145,255))
        painter.setPen(QPen(QColor(148,163,184,70),2))
        painter.drawLine(p[0],p[1]); painter.drawLine(p[4],p[5])

    def _draw_ultrasonic_waves(self, painter, body_len, t):
        y_off = self.robot_y_offset
        tip = body_len / 2
        
        speed   = 600.0   # Propagation speed (units per second)
        spacing = 200.0   # Distance between wave fronts
        
        # ── Outgoing Pings (Cyan) ──
        painter.setPen(QPen(QColor(6, 182, 212, 150), 2))
        painter.setBrush(Qt.NoBrush)
        
        shift = (t * speed) % spacing
        for i in range(4):
            r = shift + i * spacing
            if r > self.distance: continue # Stops immediately at obstacle surface
            if r < 10: continue
            
            pts = []
            for deg in range(-25, 26, 5):
                rad = math.radians(deg)
                pts.append(self.p3(r * math.sin(rad), y_off, tip + r * math.cos(rad)))
            if len(pts) > 1:
                painter.drawPolyline(QPolygonF(pts))

        # ── Returning Echoes (Rose/Red) ──
        # If the nearest obstacle is within 2500, we consider it hit by sound
        if self.distance < 2500:
            painter.setPen(QPen(QColor(244, 63, 94, 200), 2))
            
            # The echo travels backwards.
            # To sync it properly with the outbound ping hitting the wall, we could use a pure physics time formula,
            # but for visual representation, a simple retrograding wave is extremely effective.
            echo_shift = spacing - ((t * speed * 2) % spacing)
            for i in range(4):
                r_echo = self.distance - (echo_shift + i * spacing)
                if r_echo <= 0: continue
                
                pts = []
                for deg in range(-20, 21, 5):
                    rad = math.radians(deg)
                    pts.append(self.p3(r_echo * math.sin(rad), y_off, tip + r_echo * math.cos(rad)))
                if len(pts) > 1:
                    painter.drawPolyline(QPolygonF(pts))

    def _draw_ik_leg(self, painter, idx, lx, ly, lz, q1, q2, color, roff):
        kx=lx; ky=ly+self.femur_len*math.sin(q1)+roff; kz=lz+self.femur_len*math.cos(q1)
        fx=kx; fy=ky-roff+self.tibia_len*math.sin(q1+q2); fz=kz+self.tibia_len*math.cos(q1+q2)
        ph,pk,pf = self.p3(lx,ly+roff,lz),self.p3(kx,ky,kz),self.p3(fx,fy,fz)
        painter.setPen(QPen(QColor(color),11,Qt.SolidLine,Qt.RoundCap)); painter.drawLine(ph,pk)
        painter.setPen(QPen(QColor(color).lighter(150),7,Qt.SolidLine,Qt.RoundCap)); painter.drawLine(pk,pf)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor("#94a3b8"))); painter.drawEllipse(pk,7,7)
        painter.setBrush(QBrush(QColor(color))); painter.drawEllipse(pf,5,5)
        if self.show_labels:
            painter.setPen(QColor("#e2e8f0")); painter.setFont(QFont("Segoe UI",8))
            painter.drawText(ph+QPointF(10,0),f"S{idx+1}")

    def _draw_rigid_leg(self, painter, idx, lx, ly, lz, fx, fy, fz, color, roff):
        ph, pf = self.p3(lx, ly+roff, lz), self.p3(fx, fy, fz)
        painter.setPen(QPen(QColor(color),9,Qt.SolidLine,Qt.RoundCap)); painter.drawLine(ph, pf)
        painter.setPen(Qt.NoPen); painter.setBrush(QBrush(QColor(color))); painter.drawEllipse(pf,5,5)
        if self.show_labels:
            painter.setPen(QColor("#e2e8f0")); painter.setFont(QFont("Segoe UI",8))
            painter.drawText(ph+QPointF(10,0),f"S{idx+1}")

    def _draw_coordinate_axes(self, painter):
        o = self.p3(0,0,0)
        for ax,col,name in [((150,0,0),"#ef4444","X"),((0,150,0),"#10b981","Y"),((0,0,150),"#3b82f6","Z")]:
            p = self.p3(*ax); painter.setPen(QPen(QColor(col),2)); painter.drawLine(o,p)
            if self.show_labels:
                painter.setFont(QFont("Segoe UI",8,QFont.Bold))
                painter.drawText(p+QPointF(5,5),name)

    def _draw_compass(self, painter):
        painter.save()
        cx,cy,cl = 55,self.height()-55,28
        def cp(x,y,z):
            cb,sb=math.cos(self.rot_y),math.sin(self.rot_y); x,z=x*cb+z*sb,-x*sb+z*cb
            ca,sa=math.cos(self.rot_x),math.sin(self.rot_x); y,_=y*ca-z*sa,y*sa+z*ca
            return QPointF(cx+x,cy-y)
        o=QPointF(cx,cy)
        for ax,col,name in [((cl,0,0),"#ef4444","X"),((0,cl,0),"#10b981","Y"),((0,0,cl),"#3b82f6","Z")]:
            p=cp(*ax); painter.setPen(QPen(QColor(col),2)); painter.drawLine(o,p)
            painter.setFont(QFont("Segoe UI",8,QFont.Bold)); painter.drawText(p,name)
        painter.restore()
