import math
def ik(x: float, y: float, l1: float, l2: float) -> tuple:
    """

    Args:
        x: The x coordinate of the end effector
        y: The y coordinate of the end effector
        l1: The length of the first link
        l2: The length of the second link

    Returns:
        The angles of the two joints
    """
    l = x**2 + y**2
    if l > (l1 + l2)**2:
        raise ValueError("The end effector is out of reach")
    if l < (l1 - l2)**2:
        raise ValueError("The end effector is too close to the origin")

    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = min(max(cos_theta2, -1), 1)
    sin_theta2 = math.sqrt(1 - cos_theta2**2)
    sol_1 = _solution(cos_theta2, sin_theta2, x, y, l1, l2)
    sol_2 = _solution(cos_theta2, -sin_theta2, x, y, l1, l2)
    return sol_1, sol_2




def _solution(cos_theta2: float, sin_theta2: float, x: float, y: float, l1: float, l2: float) -> tuple:
    """
    Args:
        cos_theta2: The cosine of the second joint angle
        sin_theta2: The sine of the second joint angle

    Returns:
        The angles of the two joints
    """
    alpha = math.atan2(y, x)
    beta = math.atan2(l2 * sin_theta2, l1 + l2 * cos_theta2)
    theta1 = alpha - beta
    theta2 = math.atan2(sin_theta2, cos_theta2)
    return theta1, theta2
