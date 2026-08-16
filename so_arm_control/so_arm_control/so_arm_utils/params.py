from rclpy.node import Node
from rclpy.parameter import Parameter


def require_parameter(node: Node, name: str, array: bool = False):
    """Return a required parameter, raising if it's empty or unset.

    array=True requires a non-empty STRING_ARRAY; otherwise a non-empty STRING.
    """
    param_type = Parameter.Type.STRING_ARRAY if array else Parameter.Type.STRING
    default = [] if array else ''
    value = node.get_parameter_or(name, Parameter(name, param_type, default)).value
    if array:
        value = list(value)
    if not value:
        raise RuntimeError(
            f"Required parameter '{name}' is empty or unset - pass it via a parameters "
            f'yaml or -p {name}:="..." on the command line.'
        )
    return value
