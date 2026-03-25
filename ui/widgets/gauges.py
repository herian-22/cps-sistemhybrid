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
        center, side = r.center(), min(r.width(), r.height()) - 20
        
        painter.setPen(QPen(QColor("#334155"), 6))
        painter.drawArc(center.x()-side/2, center.y()-side/2, side, side, -30*16, 240*16)
        
        painter.setPen(QPen(QColor("#3b82f6"), 6))
        span = (self.angle/180)*240
        painter.drawArc(center.x()-side/2, center.y()-side/2, side, side, 210*16, -int(span)*16)
        
        painter.setPen(QColor("#e2e8f0"))
        painter.drawText(r, Qt.AlignCenter, f"{int(self.angle)}°\n{self.label}")

class DistanceWidget(QWidget):
    """Custom widget for distance sensor visualization"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.dist = 100.0
        self.setMinimumHeight(50)

    def setDistance(self, d): 
        self.dist = d
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        r = self.rect().adjusted(5, 5, -5, -5)
        painter.setBrush(QColor("#1e293b"))
        painter.drawRoundedRect(r, 5, 5)
        
        fill_w = (self.dist/100) * r.width()
        color = QColor("#ef4444" if self.dist <= 20 else "#f59e0b" if self.dist <= 30 else "#10b981")
        painter.setBrush(color)
        painter.drawRoundedRect(QRectF(r.x(), r.y(), fill_w, r.height()), 5, 5)
        
        painter.setPen(QColor("white"))
        painter.drawText(r, Qt.AlignCenter, f"{self.dist:.1f} cm")
