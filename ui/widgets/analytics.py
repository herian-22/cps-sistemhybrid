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

        # Arrows
        painter.setPen(QPen(t_col, 2))
        painter.drawLine(pw+QPointF(r,-6), pe+QPointF(-r,-6))
        painter.drawLine(pe+QPointF(-r,14), pw+QPointF(r,14))

        # Arrow heads
        painter.setBrush(QBrush(t_col))
        for tip, dirn in [(pe+QPointF(-r,-6), 1), (pw+QPointF(r,14), -1)]:
            tri = QPolygonF([tip, tip+QPointF(-8*dirn,-4), tip+QPointF(-8*dirn,4)])
            painter.drawPolygon(tri)

        # Guard labels
        painter.setPen(QColor("#94a3b8"))
        painter.setFont(QFont("Segoe UI", 7))
        mx = (pw.x()+pe.x())/2
        painter.drawText(QRectF(mx-40,h/2-22,80,14), Qt.AlignCenter, "d≤20 →")
        painter.drawText(QRectF(mx-40,h/2+10,80,14), Qt.AlignCenter, "← d≥30")

        # State bubbles
        for pos, m, name in [(pw, SystemMode.WALKING,"WALKING"),(pe, SystemMode.EVASIVE,"EVASIVE")]:
            active = (self.mode == m)
            painter.setBrush(QBrush(QColor("#3b82f6" if active else "#1e293b")))
            painter.setPen(QPen(QColor("#ffffff" if active else "#475569"), 2 if active else 1))
            painter.drawEllipse(pos, r, r)
            painter.setPen(QColor("#ffffff"))
            painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
            painter.drawText(QRectF(pos.x()-r, pos.y()-r, r*2, r*2), Qt.AlignCenter, name)


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
        w, h = rect.width(), rect.height()
        n = len(self.history[0])
        step = w / max(n-1, 1)
        for i in range(4):
            painter.setPen(QPen(QColor(colors[i]), 2))
            for j in range(n-1):
                p1 = QPointF(j*step, h-(self.history[i][j]/180)*h)
                p2 = QPointF((j+1)*step, h-(self.history[i][j+1]/180)*h)
                painter.drawLine(p1, p2)
        # Legend
        painter.setFont(QFont("Segoe UI", 8))
        for i, label in enumerate(["S1","S2","S3","S4"]):
            painter.setPen(QColor(colors[i]))
            painter.drawText(6 + i*32, 14, label)


class SignalPanel(QWidget):
    """
    Visualizes the Continuous → Discrete signal processing pipeline:
      Raw (noisy) → IIR Filtered → Quantized Event (0/1)
    This demonstrates how the hybrid automaton receives its discrete triggers
    from a noisy continuous sensor stream.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.raw_history      = [100.0]*100
        self.filtered_history = [100.0]*100
        self.quantized        = 0
        self.threshold_danger = 20.0
        self.threshold_safe   = 30.0
        self.setMinimumHeight(120)

    def setData(self, raw_h, filtered_h, quantized):
        self.raw_history      = raw_h
        self.filtered_history = filtered_h
        self.quantized        = quantized
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # Background
        painter.setBrush(QColor("#0f172a")); painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(self.rect(), 8, 8)

        if not self.raw_history: return
        n = len(self.raw_history)
        step = w / max(n-1, 1)
        max_val = 110.0  # sensor max cm

        def y_pos(val):
            return h - max(0, min(h, (val/max_val)*h))

        # Danger / safe threshold bands
        yd = y_pos(self.threshold_danger)
        ys = y_pos(self.threshold_safe)
        painter.setBrush(QColor(239, 68, 68, 25)); painter.setPen(Qt.NoPen)
        painter.drawRect(QRectF(0, yd, w, h-yd))
        painter.setBrush(QColor(245, 158, 11, 18))
        painter.drawRect(QRectF(0, ys, w, yd-ys))

        # Threshold lines
        painter.setPen(QPen(QColor(239,68,68,120), 1, Qt.DashLine))
        painter.drawLine(QPointF(0,yd), QPointF(w,yd))
        painter.setPen(QPen(QColor(245,158,11,100), 1, Qt.DashLine))
        painter.drawLine(QPointF(0,ys), QPointF(w,ys))

        # Raw signal (noisy, thin, muted)
        painter.setPen(QPen(QColor(100,116,139,160), 1))
        for j in range(n-1):
            painter.drawLine(QPointF(j*step, y_pos(self.raw_history[j])),
                             QPointF((j+1)*step, y_pos(self.raw_history[j+1])))

        # Filtered signal (smooth, bright)
        painter.setPen(QPen(QColor("#3b82f6"), 2))
        for j in range(n-1):
            painter.drawLine(QPointF(j*step, y_pos(self.filtered_history[j])),
                             QPointF((j+1)*step, y_pos(self.filtered_history[j+1])))

        # Quantized event badge (top-right)
        badge_col = QColor("#ef4444") if self.quantized else QColor("#10b981")
        painter.setBrush(QBrush(badge_col)); painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(QRectF(w-70, 6, 64, 20), 4, 4)
        painter.setPen(QColor("#ffffff"))
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        label = "DANGER [1]" if self.quantized else " SAFE  [0]"
        painter.drawText(QRectF(w-70, 6, 64, 20), Qt.AlignCenter, label)

        # Legend
        painter.setFont(QFont("Segoe UI", 7))
        painter.setPen(QColor(100,116,139,200)); painter.drawText(6, 14, "Raw")
        painter.setPen(QColor("#3b82f6"));       painter.drawText(30, 14, "Filtered")
        painter.setPen(QColor(239,68,68,180));   painter.drawText(6, h-4, f"Guard ≤{int(self.threshold_danger)}")
