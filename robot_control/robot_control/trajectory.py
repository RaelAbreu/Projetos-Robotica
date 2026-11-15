import math
from dataclasses import dataclass


@dataclass
class Reference:
    t: float
    x: float
    y: float
    dx: float
    dy: float
    ddx: float
    ddy: float
    theta: float   
    v: float       
    w: float       


class Figure8:
    def __init__(self, A: float = 1.0, B: float = 2.0, omega: float = 0.5):
        self.A, self.B, self.omega = A, B, omega

    @property
    def period(self) -> float:
        return 4.0 * math.pi / self.omega 

    def reference(self, t: float) -> Reference:
        A, B, W = self.A, self.B, self.omega
        x = A * math.sin(W * t)
        y = B * math.sin(0.5 * W * t)
        dx = A * W * math.cos(W * t)
        dy = 0.5 * B * W * math.cos(0.5 * W * t)
        ddx = -A * W * W * math.sin(W * t)
        ddy = -0.25 * B * W * W * math.sin(0.5 * W * t)
        v2 = dx * dx + dy * dy
        v = math.sqrt(v2)
        w = (dx * ddy - dy * ddx) / v2 
        return Reference(t, x, y, dx, dy, ddx, ddy, math.atan2(dy, dx), v, w)
