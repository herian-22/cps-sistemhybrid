import math
import time
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QPoint, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPolygonF
from ui.utils.projection import project

FLOOR_Y   = -120  # ground plane Y coordinate

def _solve_ik(ty, tz, L1, L2):
    D = math.sqrt(ty**2 + tz**2)
    D = max(abs(L1-L2)+0.1, min(L1+L2-0.1, D))
    cos_q2 = max(-1.0, min(1.0, (D**2-L1**2-L2**2)/(2*L1*L2)))
    q2 = math.acos(cos_q2)
    q1 = math.atan2(tz, ty) - math.atan2(L2*math.sin(q2), L1+L2*math.cos(q2))
    return q1, q2


class Robot3DWidget(QWidget):
    """
    3D visualization with:
    - Painter's Algorithm depth sorting
    - Floor-snapped obstacles (rendered from logic.obstacles list)
    - IK legs, shaded body, sensor beam, compass gizmo
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.angles    = [90.0]*4
        self.distance  = 100.0
        self.femur_len = 50.0
        self.tibia_len = 50.0
        self.robot_z   = 0.0
        self.robot_y_offset = 0.0
        self.use_ik    = True
        self.custom_obstacles = []   # custom placed + gazebo static
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
        self.robot_z       = logic.robot_z
        self.robot_y_offset= logic.robot_y_offset
        self.use_ik        = logic.use_ik
        self.custom_obstacles   = logic.custom_obstacles
        self.highlighted_obs_id = logic.highlighted_obs_id
        self.update()

    def p3(self, x, y, z):
        return project(x, y, z, self.width(), self.height(),
                       self.rot_x, self.rot_y, self.focal_length, self.zoom)

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

        # ── Custom / Map obstacles ────────────────────────────────────────────
        cc = (QColor(14,165,233,220), QColor(2,132,199,210), QColor(3,105,161,200)) # cyan
        hc = (QColor(244,63,94,220),  QColor(225,29,72,210),  QColor(190,18,60,200))  # pink highlight
        for obs in self.custom_obstacles:
            c = hc if obs.id == self.highlighted_obs_id else cc
            render_z = obs.z - self.robot_z
            if render_z < -300: continue
            dl.append((float(render_z),
                       (lambda _obs, _c, _rz: lambda p:
                        self._draw_box(p, _obs.x, FLOOR_Y, _rz,
                                       _obs.width, _obs.height, _obs.depth,
                                       _c[0], _c[1], _c[2], glow=(_obs.id==self.highlighted_obs_id)))(obs, c, render_z)))

        dl.append((1.0,  self._draw_ground_shadow))
        dl.append((0.0,  self._draw_robot_body))
        dl.append((-5.0, lambda p: self._draw_sensor_beam(p, 160, t)))

        bw, bl = 100, 160
        mounts = [(-bw/2,0,-bl/2),(bw/2,0,-bl/2),(-bw/2,0,bl/2),(bw/2,0,bl/2)]
        colors = ["#ef4444","#3b82f6","#10b981","#f59e0b"]
        for i,(lx,ly,lz) in enumerate(mounts):
            ar  = math.radians(self.angles[i]-90)
            if self.use_ik:
                ty_ =  self.femur_len*math.sin(ar)
                tz_ = -(self.femur_len*math.cos(ar)+self.tibia_len*0.8)
                try: q1, q2 = _solve_ik(ty_, tz_, self.femur_len, self.tibia_len)
                except: q1, q2 = 0.0, 0.0
                kx,ky,kz = lx, ly+self.femur_len*math.sin(q1)+self.robot_y_offset, lz+self.femur_len*math.cos(q1)
                fx,fy,fz = kx, ky-self.robot_y_offset+self.tibia_len*math.sin(q1+q2), kz+self.tibia_len*math.cos(q1+q2)
                dl.append(((lz+kz+fz)/3,
                           (lambda _i,_lx,_ly,_lz,_q1,_q2,_c,_ro:
                            lambda p: self._draw_ik_leg(p,_i,_lx,_ly,_lz,_q1,_q2,_c,_ro)
                            )(i,lx,ly,lz,q1,q2,colors[i], self.robot_y_offset)))
            else:
                # Rigid straight leg (no knee) directly down to floor target
                fx = lx
                fy = ly - (self.femur_len + self.tibia_len) * 0.8
                fz = -(self.femur_len + self.tibia_len) * math.cos(ar)
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

    def _draw_floor_grid(self, painter):
        gc = QColor(71,85,105)
        offset = self.robot_z % 150  # Infinite scrolling

        for i in range(-25,26):  # Widen base X lines from 12 to 25
            a = 90 if i==0 else 28
            painter.setPen(QPen(QColor(gc.red(),gc.green(),gc.blue(),a),1))
            painter.drawLine(self.p3(i*100,FLOOR_Y,-500), self.p3(i*100,FLOOR_Y,3500))
        for j in range(-4,26):
            a = int(max(0,90-j*3))
            painter.setPen(QPen(QColor(gc.red(),gc.green(),gc.blue(),a),1))
            z_pos = j*150 - offset
            painter.drawLine(self.p3(-2500,FLOOR_Y,z_pos), self.p3(2500,FLOOR_Y,z_pos))

    def _draw_path(self, painter):
        fy = FLOOR_Y+1
        # To make dashes scroll, we draw a very long path
        pts = [self.p3(-155,fy,-500),self.p3(155,fy,-500),
               self.p3(155,fy,2800), self.p3(-155,fy,2800)]
        painter.setBrush(QBrush(QColor(30,41,59,80))); painter.setPen(Qt.NoPen)
        painter.drawPolygon(QPolygonF(pts))
        # Optional: could make dashed line scroll too, but grid is enough for the effect

    def _draw_ground_shadow(self, painter):
        sp = self.p3(0,FLOOR_Y,0)
        painter.setPen(Qt.NoPen); painter.setBrush(QBrush(QColor(0,0,0,55)))
        painter.drawEllipse(sp, 95, 22)

    def _draw_box(self, painter, cx, base_y, cz, bw, bh, bd,
                   front_col, side_col, top_col, glow=False):
        hw, hd  = bw/2, bd/2
        top_y   = base_y + bh
        c = [
            (cx-hw,base_y,cz-hd),(cx+hw,base_y,cz-hd),(cx+hw,top_y,cz-hd),(cx-hw,top_y,cz-hd),
            (cx-hw,base_y,cz+hd),(cx+hw,base_y,cz+hd),(cx+hw,top_y,cz+hd),(cx-hw,top_y,cz+hd),
        ]
        p = [self.p3(*v) for v in c]
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

    def _draw_sensor_beam(self, painter, body_len, t):
        pulse = (math.sin(t*(200/(self.distance+5)))+1)/2
        a = int(35+90*pulse)
        if self.distance<=20: col=QColor(239,68,68,a)
        elif self.distance<=30: col=QColor(245,158,11,int(a*0.85))
        else: col=QColor(16,185,129,int(a*0.6))
        
        blen = self.distance*2; tip = body_len/2
        y_off = self.robot_y_offset
        
        pts=[(0,y_off,tip),(-40,y_off-40,tip+blen),(40,y_off-40,tip+blen),(40,y_off+40,tip+blen),(-40,y_off+40,tip+blen)]
        proj=[self.p3(*pt) for pt in pts]
        painter.setPen(QPen(col.darker(),1,Qt.DashLine)); painter.setBrush(QBrush(col))
        for i in range(1,4): painter.drawPolygon(QPolygonF([proj[0],proj[i],proj[i+1]]))
        painter.drawPolygon(QPolygonF([proj[0],proj[4],proj[1]]))

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
