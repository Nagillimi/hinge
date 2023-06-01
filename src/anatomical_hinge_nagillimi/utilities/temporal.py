import numpy as np
from anatomical_hinge_nagillimi.constants import Constants
from anatomical_hinge_nagillimi.result.temporal_result import TemporalResult

class DetectDownsampling:
    def __init__(self, sampleCount = 10, msLatencyTol = 0.1):
        self.tsBuffer = [int]
        self.k = 0
        self.index = 0
        self.avgLatency = 0
        self.sampleCount = sampleCount
        self.optimalRange = {
            "low": Constants.OPTIMAL_ALGORITHM_LATENCY_MS - msLatencyTol,
            "high": Constants.OPTIMAL_ALGORITHM_LATENCY_MS + msLatencyTol,
        }


    def update(self, ts: int) -> TemporalResult:
        self.tsBuffer.append(ts)
        if self.tsBuffer.count == self.sampleCount:
            self.avgLatency = np.average(self.tsBuffer)
            return self.compute()
        else:
            return TemporalResult.NOT_ENOUGH_SAMPLES


    def compute(self) -> TemporalResult:
        if self.avgLatency > self.optimalRange.get("high"):
            return TemporalResult.BELOW
        elif self.optimalRange.get("low") < self.avgLatency < self.optimalRange.get("high"):
            return TemporalResult.OPTIMAL
        else:
            return TemporalResult.ABOVE
