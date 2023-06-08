class HistoricNumber:
    def __init__(self) -> None:
        self.current = 0.
        self.past    = 0.
        self.past2   = 0.

    def shift(self, newNumber: float):
        self.past2   = self.past
        self.past    = self.current
        self.current = newNumber

    def delta(self):
        return self.current - self.past
    
    def delta2(self):
        return self.current - self.past2
        

class HistoricPoint:
    def __init__(self) -> None:
        self.x = HistoricNumber()
        self.y = HistoricNumber()
        self.z = HistoricNumber()

    def shift(self, newPoint: list[float]):
        self.x.shift(newPoint[0])
        self.y.shift(newPoint[1])
        self.z.shift(newPoint[2])

    def current(self):
        return [self.x.current, self.y.current, self.z.current]

    def past(self):
        return [self.x.past, self.y.past, self.z.past]

    def past2(self):
        return [self.x.past2, self.y.past2, self.z.past2]

    def delta(self):
        return [self.x.delta(), self.y.delta(), self.z.delta()]
    
    def delta2(self):
        return [self.x.delta2(), self.y.delta2(), self.z.delta2()]
    