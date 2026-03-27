import math
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPolygonF
from core.logic import SystemMode

class StateDiagramWidget(QWidget):
    """Hybrid Automaton state diagram with animated transition arrow."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.mode = SystemMode.WALKING
        self.transitioning = False
        self.setMinimumSize(260, 130)

    def setMode(self, mode, transitioning):
        self.mode, self.transitioning = mode, transitioning
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h, r = self.width(), self.height(), 38
        pw = QPointF(w*0.25, h/2)
        pe = QPointF(w*0.75, h/2)
        t_col = QColor("#f59e0b") if self.transitioning else QColor("#475569")

        painter.setPen(QPen(t_col, 2))
        painter.drawLine(pw+QPointF(r,-6), pe+QPointF(-r,-6))
        painter.drawLine(pe+QPointF(-r,14), pw+QPointF(r,14))

        painter.setBrush(QBrush(t_col))
        for tip, dirn in [(pe+QPointF(-r,-6), 1), (pw+QPointF(r,14), -1)]:
            tri = QPolygonF([tip, tip+QPointF(-8*dirn,-4), tip+QPointF(-8*dirn,4)])
            painter.drawPolygon(tri)

        painter.setPen(QColor("#94a3b8"))
        painter.setFont(QFont("Segoe UI", 7))
        mx = (pw.x()+pe.x())/2
        painter.drawText(QRectF(mx-40,h/2-22,80,14), Qt.AlignCenter, "d≤20 →")
        painter.drawText(QRectF(mx-40,h/2+10,80,14), Qt.AlignCenter, "← d≥30")

        for pos, m, name in [(pw, SystemMode.WALKING,"WALKING"),(pe, SystemMode.EVASIVE,"EVASIVE")]:
            active = (self.mode == m)
            painter.setBrush(QBrush(QColor("#3b82f6" if active else "#1e293b")))
            painter.setPen(QPen(QColor("#ffffff" if active else "#475569"), 2 if active else 1))
            painter.drawEllipse(pos, r, r)
            painter.setPen(QColor("#ffffff"))
            painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
            painter.drawText(QRectF(pos.x()-r, pos.y()-r, r*2, r*2), Qt.AlignCenter, name)

# ── Base Analysis Widget ───────────────────────────────────────────────────
class AnalysisWidget(QWidget):
    def __init__(self, title, analysis, parent=None):
        super().__init__(parent)
        self.title = title
        self.analysis = analysis
        self.setMinimumHeight(240)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        r = self.rect()
        
        # Background card
        painter.setBrush(QColor("#1e293b"))
        painter.setPen(QPen(QColor("#334155"), 1))
        painter.drawRoundedRect(r.adjusted(1,1,-1,-1), 10, 10)
        
        # Header
        painter.setPen(QColor("#3b82f6")) 
        painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
        painter.drawText(20, 30, self.title.upper())
        
        # Analysis Footer
        painter.setPen(QColor("#94a3b8"))
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(QRectF(20, r.height()-50, r.width()-40, 45), 
                         Qt.AlignLeft | Qt.AlignVCenter | Qt.TextWordWrap, 
                         f"Analisis: {self.analysis}")

        # Chart Box
        self.chart_rect = r.adjusted(20, 45, -20, -60)
        painter.setBrush(QColor("#0f172a"))
        painter.setPen(QPen(QColor("#334155"), 1))
        painter.drawRoundedRect(self.chart_rect, 6, 6)
        
        self.drawChart(painter, self.chart_rect)

    def drawChart(self, painter, rect):
        pass

# ── Specialized Analysis Widgets ───────────────────────────────────────────

class SignalAnalysisWidget(AnalysisWidget):
    def setData(self, raw, filtered):
        self.raw, self.filtered = raw, filtered
        self.update()

    def drawChart(self, painter, rect):
        if not hasattr(self, 'raw') or not self.raw: return
        w, h = rect.width(), rect.height()
        n = len(self.raw)
        step = w / max(n-1, 1)
        max_val = 110.0
        def y_pos(v): 
            clamped = max(0, min(max_val, v))
            return rect.bottom() - (clamped / max_val) * h

        # Raw (noisy)
        painter.setPen(QPen(QColor(100,116,139,120), 1))
        for j in range(n-1):
            painter.drawLine(QPointF(rect.x() + j*step, y_pos(self.raw[j])),
                             QPointF(rect.x() + (j+1)*step, y_pos(self.raw[j+1])))
        # Filtered (smooth)
        painter.setPen(QPen(QColor("#3b82f6"), 2))
        for j in range(n-1):
            painter.drawLine(QPointF(rect.x() + j*step, y_pos(self.filtered[j])),
                             QPointF(rect.x() + (j+1)*step, y_pos(self.filtered[j+1])))

class ModeTransitionWidget(AnalysisWidget):
    def setData(self, modes):
        self.modes = modes
        self.update()

    def drawChart(self, painter, rect):
        if not hasattr(self, 'modes') or not self.modes: return
        w, h = rect.width(), rect.height()
        n = len(self.modes)
        step = w / max(n-1, 1)
        pad = 15
        
        # Step chart
        painter.setPen(QPen(QColor("#10b981"), 2))
        for j in range(n-1):
            y1 = rect.top() + pad if self.modes[j] == 1 else rect.bottom() - pad
            y2 = rect.top() + pad if self.modes[j+1] == 1 else rect.bottom() - pad
            # Vertical jump
            painter.drawLine(QPointF(rect.x() + j*step, y1), QPointF(rect.x() + (j+1)*step, y1))
            if y1 != y2:
                painter.drawLine(QPointF(rect.x() + (j+1)*step, y1), QPointF(rect.x() + (j+1)*step, y2))
        
        painter.setPen(QColor("#64748b"))
        painter.setFont(QFont("Segoe UI", 7, QFont.Bold))
        painter.drawText(rect.x()+5, rect.top()+15, "WALKING (1)")
        painter.drawText(rect.x()+5, rect.bottom()-5, "EVASIVE (0)")

class TrajectoryWidget(AnalysisWidget):
    def setData(self, zs):
        self.zs = zs
        self.update()

    def drawChart(self, painter, rect):
        if not hasattr(self, 'zs') or not self.zs: return
        w, h = rect.width(), rect.height()
        n = len(self.zs)
        step = w / max(n-1, 1)
        z_min, z_max = min(self.zs), max(self.zs)
        span = max(1.0, z_max - z_min)
        
        painter.setPen(QPen(QColor("#f59e0b"), 2))
        for j in range(n-1):
            y1 = rect.bottom() - ((self.zs[j] - z_min) / span) * (h - 20) - 10
            y2 = rect.bottom() - ((self.zs[j+1] - z_min) / span) * (h - 20) - 10
            painter.drawLine(QPointF(rect.x() + j*step, y1), QPointF(rect.x() + (j+1)*step, y2))
            
        painter.setPen(QColor("#64748b"))
        painter.drawText(rect.x()+5, rect.bottom()-5, f"Z_min: {int(z_min)}")
        painter.drawText(rect.x()+5, rect.top()+12, f"Z_max: {int(z_max)}")

class ServoResponseWidget(AnalysisWidget):
    def setData(self, servos):
        self.s1, self.s2 = servos[0], servos[1]
        self.update()

    def drawChart(self, painter, rect):
        if not hasattr(self, 's1') or not self.s1: return
        w, h = rect.width(), rect.height()
        n = len(self.s1)
        step = w / max(n-1, 1)
        
        for i, data in enumerate([self.s1, self.s2]):
            painter.setPen(QPen(QColor("#ef4444" if i==0 else "#3b82f6"), 2))
            for j in range(n-1):
                y1 = rect.bottom() - (data[j]/180.0) * h
                y2 = rect.bottom() - (data[j+1]/180.0) * h
                painter.drawLine(QPointF(rect.x() + j*step, y1), QPointF(rect.x() + (j+1)*step, y2))
        
        painter.setFont(QFont("Segoe UI", 7))
        painter.setPen(QColor("#ef4444")); painter.drawText(rect.x()+5, rect.top()+12, "Servo 1")
        painter.setPen(QColor("#3b82f6")); painter.drawText(rect.x()+5, rect.top()+24, "Servo 2")

# ── Old Widgets (kept for compatibility or small dashboard strips) ──────────

class RealTimePlotWidget(QWidget):
    """Scrolling graph of servo angles."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.history = [[] for _ in range(4)]
        self.setMinimumHeight(100)

    def setData(self, h):
        self.history = h
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.rect()
        painter.setBrush(QColor("#0f172a")); painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(rect, 8, 8)
        if not self.history[0]: return
        colors = ["#ef4444","#3b82f6","#10b981","#f59e0b"]
        pad = 6
        w, h = rect.width() - pad*2, rect.height() - pad*2
        n = len(self.history[0])
        step = w / max(n-1, 1)
        for i in range(4):
            painter.setPen(QPen(QColor(colors[i]), 2))
            for j in range(n-1):
                y1 = max(pad, min(rect.height()-pad, h - (self.history[i][j]/180.0)*h + pad))
                y2 = max(pad, min(rect.height()-pad, h - (self.history[i][j+1]/180.0)*h + pad))
                painter.drawLine(QPointF(j*step + pad, y1), QPointF((j+1)*step + pad, y2))

class SignalPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.raw_history, self.filtered_history, self.quantized = [100.0]*100, [100.0]*100, 0
        self.threshold_danger, self.threshold_safe = 20.0, 30.0
        self.setMinimumHeight(120)

    def setData(self, raw_h, filtered_h, quantized):
        self.raw_history, self.filtered_history, self.quantized = raw_h, filtered_h, quantized
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        painter.setBrush(QColor("#0f172a")); painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(self.rect(), 8, 8)
        if not self.raw_history: return
        n, step, max_val = len(self.raw_history), self.width()/99, 110.0
        def y_pos(val): cl=max(0.0, min(max_val, val)); return h - (cl/max_val)*h
        yd, ys = y_pos(self.threshold_danger), y_pos(self.threshold_safe)
        painter.setBrush(QColor(239,68,68,25)); painter.drawRect(QRectF(0, yd, w, h-yd))
        painter.setBrush(QColor(245,158,11,18)); painter.drawRect(QRectF(0, ys, w, yd-ys))
        painter.setPen(QPen(QColor(239,68,68,120),1,Qt.DashLine)); painter.drawLine(QPointF(0,yd),QPointF(w,yd))
        painter.setPen(QPen(QColor(245,158,11,100),1,Qt.DashLine)); painter.drawLine(QPointF(0,ys),QPointF(w,ys))
        painter.setPen(QPen(QColor(100,116,139,160),1))
        for j in range(n-1): painter.drawLine(QPointF(j*step,y_pos(self.raw_history[j])), QPointF((j+1)*step,y_pos(self.raw_history[j+1])))
        painter.setPen(QPen(QColor("#3b82f6"),2))
        for j in range(n-1): painter.drawLine(QPointF(j*step,y_pos(self.filtered_history[j])), QPointF((j+1)*step,y_pos(self.filtered_history[j+1])))
        badge_col = QColor("#ef4444") if self.quantized else QColor("#10b981")
        painter.setBrush(QBrush(badge_col)); painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(QRectF(w-70,6,64,20),4,4); painter.setPen(QColor("#ffffff")); painter.setFont(QFont("Segoe UI",8,QFont.Bold))
        painter.drawText(QRectF(w-70,6,64,20),Qt.AlignCenter,"DANGER [1]" if self.quantized else " SAFE  [0]")
