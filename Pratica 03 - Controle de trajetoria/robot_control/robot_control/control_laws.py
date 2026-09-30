import math
from dataclasses import dataclass
from typing import Dict, Tuple

from .pid import ErrorPID, Gains
from .trajectory import Figure8, Reference
from .utils import wrap_angle


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def _sym(gains: Gains, limit: float) -> ErrorPID:
    return ErrorPID(gains, -limit, limit)


# ---------------------------------------------------------------------------
# 1.1  Controle de pose contínuo 
# ---------------------------------------------------------------------------
class ContinuousPoseController:
    def __init__(self, gains: Dict[str, Gains], pos_tol: float, yaw_tol: float,
                 v_max: float, w_max: float):
        self.pos_tol, self.yaw_tol = pos_tol, yaw_tol
        self.pid_rho = ErrorPID(gains['rho'], 0.0, v_max)
        self.pid_alpha = _sym(gains['alpha'], w_max)
        self.pid_beta = _sym(gains['beta'], w_max)
        self.pid_yaw = _sym(gains['yaw'], w_max)
        self.w_max = w_max

    def reset(self) -> None:
        for p in (self.pid_rho, self.pid_alpha, self.pid_beta, self.pid_yaw):
            p.reset()

    def compute(self, pose: Pose2D, goal: Pose2D, dt: float
                ) -> Tuple[float, float, bool]:
        dx, dy = goal.x - pose.x, goal.y - pose.y
        rho = math.hypot(dx, dy)

        if rho < self.pos_tol:  # chegou em (x, y): só falta a orientação
            e_yaw = wrap_angle(goal.yaw - pose.yaw)
            if abs(e_yaw) < self.yaw_tol:
                return 0.0, 0.0, True
            return 0.0, self.pid_yaw(e_yaw, dt), False

        los = math.atan2(dy, dx)
        alpha = wrap_angle(los - pose.yaw)
        beta = wrap_angle(goal.yaw - los)

        v = self.pid_rho(rho, dt) * max(0.0, math.cos(alpha))
        w = self.pid_alpha(alpha, dt) + self.pid_beta(beta, dt)
        w = max(-self.w_max, min(self.w_max, w))
        return v, w, False


# ---------------------------------------------------------------------------
# 1.1 (bônus)  Controle de pose com três manobras
# ---------------------------------------------------------------------------
class ThreeManeuverPoseController:
    """Rotação -> translação -> rotação, com máquina de estados."""

    ALIGN, TRANSLATE, FINAL, DONE = 'ALIGN', 'TRANSLATE', 'FINAL', 'DONE'

    def __init__(self, gains: Dict[str, Gains], pos_tol: float, yaw_tol: float,
                 align_tol: float, v_max: float, w_max: float):
        self.pos_tol, self.yaw_tol, self.align_tol = pos_tol, yaw_tol, align_tol
        self.pid_align = _sym(gains['align'], w_max)
        self.pid_rho = ErrorPID(gains['rho'], 0.0, v_max)
        self.pid_heading = _sym(gains['heading'], w_max)
        self.pid_yaw = _sym(gains['yaw'], w_max)
        self.state = self.ALIGN

    def reset(self) -> None:
        """Deve ser chamado a cada novo objetivo."""
        self.state = self.ALIGN
        for p in (self.pid_align, self.pid_rho, self.pid_heading, self.pid_yaw):
            p.reset()

    def _go(self, state: str, *pids: ErrorPID) -> None:
        self.state = state
        for p in pids:
            p.reset()

    def compute(self, pose: Pose2D, goal: Pose2D, dt: float
                ) -> Tuple[float, float, bool]:
        dx, dy = goal.x - pose.x, goal.y - pose.y
        rho = math.hypot(dx, dy)
        los = math.atan2(dy, dx)

        if self.state == self.ALIGN:                       # manobra 1
            if rho < self.pos_tol:
                self._go(self.FINAL, self.pid_yaw)
            else:
                e = wrap_angle(los - pose.yaw)
                if abs(e) < self.align_tol:
                    self._go(self.TRANSLATE, self.pid_rho, self.pid_heading)
                else:
                    return 0.0, self.pid_align(e, dt), False

        if self.state == self.TRANSLATE:                   # manobra 2
            if rho < self.pos_tol:
                self._go(self.FINAL, self.pid_yaw)
            else:
                e = wrap_angle(los - pose.yaw)
                v = self.pid_rho(rho, dt) * max(0.0, math.cos(e))
                return v, self.pid_heading(e, dt), False

        if self.state == self.FINAL:                       # manobra 3
            e = wrap_angle(goal.yaw - pose.yaw)
            if abs(e) < self.yaw_tol:
                self.state = self.DONE
            else:
                return 0.0, self.pid_yaw(e, dt), False

        return 0.0, 0.0, True


# ---------------------------------------------------------------------------
# 2.1  Seguimento de trajetória 
# ---------------------------------------------------------------------------
@dataclass
class TrackingError:
    ex_w: float   
    ey_w: float
    ex: float     
    ey: float     
    eth: float    


class TrajectoryTracker:
    def __init__(self, traj: Figure8, mode: str, k_ff: float,
                 gains: Dict[str, Gains], v_max: float, w_max: float):
        if mode not in ('open_loop', 'feedback'):
            raise ValueError("mode deve ser 'open_loop' ou 'feedback'")
        self.traj, self.mode, self.k_ff = traj, mode, k_ff
        self.v_max, self.w_max = v_max, w_max
        self.pid_x = _sym(gains['x'], 2 * v_max)
        self.pid_y = _sym(gains['y'], 2 * w_max)
        self.pid_th = _sym(gains['theta'], 2 * w_max)

    def reset(self) -> None:
        for p in (self.pid_x, self.pid_y, self.pid_th):
            p.reset()

    def compute(self, t: float, pose: Pose2D, dt: float
                ) -> Tuple[float, float, Reference, TrackingError]:
        ref = self.traj.reference(t)
        exw, eyw = ref.x - pose.x, ref.y - pose.y
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        err = TrackingError(exw, eyw, c * exw + s * eyw, -s * exw + c * eyw,
                            wrap_angle(ref.theta - pose.yaw))

        if self.mode == 'open_loop':
            v, w = ref.v, ref.w
        else:
            v = self.k_ff * ref.v * math.cos(err.eth) + self.pid_x(err.ex, dt)
            w = (self.k_ff * ref.w + self.pid_y(err.ey, dt)
                 + self.pid_th(err.eth, dt))

        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))
        return v, w, ref, err


class RMSE:
    def __init__(self):
        self.n = 0
        self._pos = 0.0
        self._yaw = 0.0

    def add(self, e: TrackingError) -> None:
        self.n += 1
        self._pos += e.ex_w ** 2 + e.ey_w ** 2
        self._yaw += e.eth ** 2

    @property
    def pos(self) -> float:
        return math.sqrt(self._pos / self.n) if self.n else 0.0

    @property
    def yaw(self) -> float:
        return math.sqrt(self._yaw / self.n) if self.n else 0.0
