from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont

class ServoWidget(QWidget):
    """Custom widget to visualize a single servo angle"""
    def __init__(self, label, parent=None):
        super().__init__(parent)
        self.label, self.angle = label, 90.0
        self.setMinimumSize(100, 100)

    def setAngle(self, a):
        self.angle = a
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        r = self.rect()
        center = r.center()
        side = min(r.width(), r.height()) - 20

        # Background circle
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#1e293b"))
        painter.drawEllipse(center, side//2, side//2)

        # Track arc
        painter.setPen(QPen(QColor("#334155"), 6))
        painter.setBrush(Qt.NoBrush)
        painter.drawArc(int(center.x()-side/2), int(center.y()-side/2), side, side, -30*16, 240*16)

        # Value arc – Bug fix: clamp angle to [0,180] before computing span
        clamped = max(0.0, min(180.0, self.angle))
        span = (clamped / 180.0) * 240
        painter.setPen(QPen(QColor("#3b82f6"), 6))
        painter.drawArc(int(center.x()-side/2), int(center.y()-side/2), side, side, 210*16, -int(span)*16)

        # Text
        painter.setPen(QColor("#e2e8f0"))
        painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
        painter.drawText(r, Qt.AlignCenter, f"{int(self.angle)}°\n{self.label}")

class DistanceWidget(QWidget):
    """Custom widget for distance sensor visualization"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.dist = 100.0
        self.setMinimumHeight(50)

    def setDistance(self, d):
        # Bug fix: clamp distance so bar never overflows or underflows
        self.dist = max(0.0, min(d, 100.0))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        r = self.rect().adjusted(5, 5, -5, -5)

        # Background
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#1e293b"))
        painter.drawRoundedRect(r, 5, 5)

        # Fill bar – Bug fix: fill_w clamped to [0, r.width()]
        fill_w = max(0.0, min((self.dist / 100.0) * r.width(), float(r.width())))
        color = QColor("#ef4444" if self.dist <= 20 else "#f59e0b" if self.dist <= 30 else "#10b981")
        painter.setBrush(color)
        if fill_w > 0:
            painter.drawRoundedRect(QRectF(r.x(), r.y(), fill_w, r.height()), 5, 5)

        # Text
        painter.setPen(QColor("white"))
        painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
        painter.drawText(r, Qt.AlignCenter, f"{self.dist:.1f} cm")
