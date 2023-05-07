
class HistoricNumber:
    def __init__(self) -> None:
        self.current: float = 0.0
        self.past: float    = 0.0
        self.past2: float   = 0.0

    def shift(self, newNumber: float):
        self.past2   = self.past
        self.past    = self.current
        self.current = newNumber
        

class HistoricPoint:
    def __init__(self) -> None:
        self.x = HistoricNumber()
        self.y = HistoricNumber()
        self.z = HistoricNumber()

    def shift(self, newPoint: list[float]):
        self.x.shift(newPoint[0])
        self.y.shift(newPoint[1])
        self.z.shift(newPoint[2])

    def current(self) -> list[float]:
        return [self.x.current, self.y.current, self.z.current]

    def past(self) -> list[float]:
        return [self.x.past, self.y.past, self.z.past]

    def past2(self) -> list[float]:
        return [self.x.past2, self.y.past2, self.z.past2]
