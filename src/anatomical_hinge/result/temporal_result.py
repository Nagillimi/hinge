from enum import Enum

class TemporalResult(Enum):
    OPTIMAL = "Current signal sampling is optimal for calibration"
    ABOVE = "Downsampled signal sampling since it's above optimal values"
    BELOW = "No downsampling required, signal sampling is below minimum threshold. Uncharted territory."
    NOT_ENOUGH_SAMPLES = "Not enough signal data yet"
    