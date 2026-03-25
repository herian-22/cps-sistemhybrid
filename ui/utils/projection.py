import math
from PySide6.QtCore import QPointF

def project(x, y, z, width, height, rot_x=0.5, rot_y=0.15, focal_length=500, zoom=800):
    """Perspective projection with dynamic camera parameters"""
    # Yaw rotation (Orbit)
    cos_b, sin_b = math.cos(rot_y), math.sin(rot_y)
    x, z = x * cos_b + z * sin_b, -x * sin_b + z * cos_b
    
    # Pitch rotation (Tilt)
    cos_a, sin_a = math.cos(rot_x), math.sin(rot_x)
    y, z = y * cos_a - z * sin_a, y * sin_a + z * cos_a
    
    # Depth offset
    z_final = z + zoom
    if z_final <= 1: z_final = 1
    
    scale = focal_length / z_final
    px = x * scale + width / 2
    py = -y * scale + height / 2 + 100
    return QPointF(px, py)
