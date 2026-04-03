# four_bar.py
#
# This file contains functions for simulating the PIP/DIP angle relationship, used in the 
# powerhouse_finger.urdf.xacro file.
import math
# -----------------------------
# Geometry (mm) + mount angles (deg)
# -----------------------------
d = 40.0        # L_PD
L = 10.0        # L_PB = L_DB
b = 36.293      # L_BB

phiP0 = math.radians(-109.736)
phiD0 = math.radians(160.264)

def clamp(x, lo=-1.0, hi=1.0):
    return min(max(x, lo), hi)

def solve_qD_from_qP(qP_deg, branch=+1):
    """
    Closed-form closure solution for qD given qP:
      p = phiP0 + qP
      u = psi ± acos(k/R)
      qD = phiD0 - u
    branch selects assembly mode (+1/-1). For your linkage, +1 matches qD≈qP at 0/45/90.
    """
    qP = math.radians(qP_deg)
    p  = phiP0 + qP

    A = d - L*math.cos(p)
    B = -L*math.sin(p)

    k = (b**2 - ((d - L*math.cos(p))**2 + (L*math.sin(p))**2 + L**2)) / (2*L)
    R = math.sqrt(A**2 + B**2)
    psi = math.atan2(B, A)

    u  = psi + branch*math.acos(clamp(k / R))
    qD = phiD0 - u
    return qD / qP