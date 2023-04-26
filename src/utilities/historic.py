class HistoricNumber:
    current = 0
    past = 0
    past2 = 0

    def shift(self, newTs):
        self.past2 = self.past
        self.past = self.current
        self.current = newTs
        
class HistoricPoint:
    x = HistoricNumber()
    y = HistoricNumber()
    z = HistoricNumber()

    def shift(self, newPoint: list):
        self.x.shift(x)
        self.y.shift(y)
        self.z.shift(z)