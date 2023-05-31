import json

# Contains JSON encoded data for all the current details of the algorithm
class Details:
    def __init__(self) -> None:
        pass

    def createReportJSON(self, contents) -> str:
        return json.dumps(contents)