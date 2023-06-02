from typing import List

# Data object to expose to the API, simple object of lists.
# Internally converted for better abstraction.
class Data:
    def __init__(
            self,
            ts: int,
            a1: List[float],
            g1: List[float],
            a2: List[float],
            g2: List[float]) -> None:
        self.ts = ts
        self.a1 = a1
        self.g1 = g1
        self.a2 = a2
        self.g2 = g2
