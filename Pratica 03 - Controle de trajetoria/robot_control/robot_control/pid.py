from dataclasses import dataclass
from simple_pid import PID


@dataclass
class Gains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


class ErrorPID:
    def __init__(self, gains: Gains, lo: float, hi: float):
        self._pid = PID(
            gains.kp, gains.ki, gains.kd,
            setpoint=0.0,
            sample_time=None,               
            output_limits=(lo, hi),
            differential_on_measurement=False,  
        )

    def __call__(self, error: float, dt: float) -> float:
        return float(self._pid(-error, dt=max(dt, 1e-6)))

    def reset(self) -> None:
        self._pid.reset()
