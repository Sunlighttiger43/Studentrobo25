import random


class PIDController:
    prev = 0
    total = 0

    def __init__(self, sp: float, p: float = 0, i: float = 0, d: float = 0):
        self.sp = sp
        self.p = p
        self.i = i
        self.d = d

    def getCorrection(self, pv: float) -> float:
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
