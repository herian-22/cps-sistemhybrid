import math
import numpy as np
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QPolygonF
from core.pathfinding import CELL_SIZE, GRID_SIZE, OFFSET


class RVizMapWidget(QWidget):
    """
    RViz-standard 2D Occupancy Grid Map.
    Gray #7F7F7F: Unknown, White #FFFFFF: Free, Black #000000: Obstacle.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.logic = None
        self.setMinimumSize(260, 200)

    def setParams(self, logic):
        self.logic = logic
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.Antialiasing, False)
            
            # Guard clause
            if not self.logic or not hasattr(self.logic, 'occupancy_grid'):
                return

            grid_obj = self.logic.occupancy_grid
            if grid_obj is None or not hasattr(grid_obj, 'grid') or grid_obj.grid is None:
                return
                
            grid = grid_obj.grid
            w, h = self.width(), self.height()
            
            # Draw background
            painter.fillRect(self.rect(), QColor("#0f172a"))
            
            # Grid settings
            rows, cols = grid.shape
            cell_w, cell_h = w / cols, h / rows
            
            # Draw occupied cells (optimized)
            painter.setPen(Qt.NoPen)
            occupied_color = QColor("#3b82f6")
            
            # Scale world coordinate system to fit widget
            occupied_color = QColor("#475569") # Slate 600
            free_color = QColor("#e2e8f0")     # Slate 200 (Free Space)
            
            # Draw cleared (free) space
            fy_coords, fx_coords = np.where(grid < 0.3)
            for y, x in zip(fy_coords, fx_coords):
                painter.fillRect(int(x * cell_w), int(y * cell_h), int(cell_w)+1, int(cell_h)+1, free_color)

            # Draw occupied cells
            oy_coords, ox_coords = np.where(grid > 0.5)
            for y, x in zip(oy_coords, ox_coords):
                painter.fillRect(int(x * cell_w), int(y * cell_h), int(cell_w)+1, int(cell_h)+1, occupied_color)

            # Draw robot
            rx, rz = self.logic.robot_x, self.logic.robot_z
            rot = self.logic.robot_rotation
            
            # Map world to grid
            # rx, rz are in world units
            res = getattr(grid_obj, 'RESOLUTION', CELL_SIZE)
            off = getattr(grid_obj, 'OFFSET', OFFSET)
            
            gx = (rx / res) + off
            gz = (rz / res) + off
            
            px, py = gx * cell_w, gz * cell_h
            
            painter.save()
            painter.translate(px, py)
            painter.rotate(rot)
            
            # Robot body (Arrow style)
            painter.setBrush(QColor("#10b981"))
            poly = QPolygonF([QPointF(0, -10), QPointF(-6, 8), QPointF(0, 4), QPointF(6, 8)])
            painter.drawPolygon(poly)
            painter.restore()

            # Draw Planned Path
            if hasattr(self.logic, 'planned_path') and self.logic.planned_path:
                path_pen = QPen(QColor("#f59e0b"), 2, Qt.DashLine)
                painter.setPen(path_pen)
                last_pt = (px, py)
                for wp in self.logic.planned_path:
                    wgx = (wp[0] / res) + off
                    wgz = (wp[1] / res) + off
                    curr_pt = (wgx * cell_w, wgz * cell_h)
                    painter.drawLine(last_pt[0], last_pt[1], curr_pt[0], curr_pt[1])
                    last_pt = curr_pt
        finally:
            painter.end()
