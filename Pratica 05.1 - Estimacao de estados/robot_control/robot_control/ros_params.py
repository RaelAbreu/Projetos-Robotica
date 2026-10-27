from rcl_interfaces.msg import ParameterDescriptor

from .pid import Gains

_DYN = ParameterDescriptor(dynamic_typing=True)


def param(node, name, default):
    return node.declare_parameter(name, default, _DYN).value


def fparam(node, name, default) -> float:
    return float(param(node, name, default))


def farray(node, name, default):
    return [float(v) for v in param(node, name, default)]


def gains_param(node, prefix, kp, ki=0.0, kd=0.0) -> Gains:
    """Lê `<prefix>.kp/.ki/.kd` (YAML aninhado)."""
    return Gains(fparam(node, f'{prefix}.kp', kp),
                 fparam(node, f'{prefix}.ki', ki),
                 fparam(node, f'{prefix}.kd', kd))
