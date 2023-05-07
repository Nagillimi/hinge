import json
from anatomical_hinge import AnatomicalHinge

# Contains JSON encoded data for all the current details of the algorithm
class Details:
    def __init__(self) -> None:
        pass

    def createReportJSON(self, anatomicalHinge: AnatomicalHinge) -> str:
        return json.dumps(anatomicalHinge)