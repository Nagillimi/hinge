# Data object to expose to the API, simple object of lists.
# Internally converted for better abstraction.
class Data:
    def __init__(self) -> None:
        self.ts = int()
        self.a1 = []
        self.g1 = []
        self.a2 = []
        self.g2 = []
