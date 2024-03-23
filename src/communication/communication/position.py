class Position:
    """
    Position class that get published as a JSON header to MQTT topic.
    """
    def __init__(self):
        self.easting = 0.0
        self.northing = 0.0
        self.elevation = 0.0
        self.extras = []
