import math

def tilt_compensated_heading(mag_x, mag_y, mag_z, accel_x, accel_y, accel_z):
    """
    Returns heading in degrees (0-360), compensated for tilt.
    All axes in same units (normalized or raw scaled).
    """
    # Normalize accelerometer vector
    norm = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    ax = accel_x / norm
    ay = accel_y / norm
    az = accel_z / norm

    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    roll = math.atan2(ay, az)
    
    # Tilt compensation
    Xh = mag_x * math.cos(ay) + mag_z * math.sin(ay)
    Yh = mag_x * math.sin(ax) * math.sin(ay) + mag_y * math.cos(ax) - mag_z * math.sin(ax) * math.cos(ay)

    heading = math.atan2(Yh, Xh)
    heading_deg = math.degrees(heading)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

