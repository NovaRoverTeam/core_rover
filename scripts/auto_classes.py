#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# DesPosClass:
#    Creates a class for the GPS coords given by the competition.
#    This is updated by subscribing to the navigation node
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
class WaypointClass(object):

    def __init__(self, lat, lng):
        self.latitude = lat
        self.longitude = lng

    def setCoords(self, lat, lng):
        self.latitude = lat
        self.longitude = lng

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# RoveyPosClass:
#    Creates a class for the location of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
class RoveyPosClass(object):

    def __init__(self, lat, lng, x, z):
        self.latitude = lat
        self.longitude = lng
        self.x = x
        self.z = z

    def setCoords(self, lat, lng):
        self.latitude = lat
        self.longitude = lng

    def setOrientation(self, x, z):
        self.x = x
        self.z = z
