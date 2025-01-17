# pid.py

import random

class PIDController:
    """
    A simple PID (Proportional, Integral, Derivative) controller implementation.

    This controller calculates a correction value (control signal) to reduce
    the error between a setpoint (SP) and the current process variable (PV).
    """
    prev = 0
    total = 0

    def __init__(self, sp: float, p: float = 0, i: float = 0, d: float = 0):
        """
        Initialises the PIDController with given gain values.

        Parameters:
            sp (float): The setpoint (target value).
            p (float, optional): Proportional gain. Defaults to 0.
            i (float, optional): Integral gain. Defaults to 0.
            d (float, optional): Derivative gain. Defaults to 0.
        """
        self.sp = sp
        self.p = p
        self.i = i
        self.d = d

    def getCorrection(self, pv: float) -> float:
        """
        Computes the control signal (correction) based on the current process variable.

        The control signal is calculated as:
            correction = P * error + I * accumulated_error + D * (error - previous_error)

        Parameters:
            pv (float): The current process variable.

        Returns:
            float: The calculated correction value.
        """
        error = self.sp - pv
        self.total += error
        res = (self.p * error) + (self.i * self.total) + (self.d * (error - self.prev))
        self.prev = error
        return res

    def reset(
        self,
        sp: float | None = None,
        p: float | None = None,
        i: float | None = None,
        d: float | None = None,
    ):
        """
        Resets the PID controller state and optionally updates gains.

        Parameters:
            sp (float, optional): The new setpoint. If None, it remains unchanged.
            p (float, optional): The new proportional gain. If None, it remains unchanged.
            i (float, optional): The new integral gain. If None, it remains unchanged.
            d (float, optional): The new derivative gain. If None, it remains unchanged.
        """
        self.prev = 0
        self.total = 0
        if sp != None:
            self.sp = sp
        if p != None:
            self.p = p
        if i != None:
            self.i = i
        if d != None:
            self.d = d


if __name__ == "__main__":
    sp = 0
    pv = 30
    pid = PIDController(sp, 0.3, 0.3, 0.3)
    while abs(pv - sp) > 1:
        print(pv)
        pv += random.randint(-10, 10)
        pv += pid.getCorrection(pv)
    print(pv)
