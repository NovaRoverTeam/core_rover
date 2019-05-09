import math

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
    def __str__(self):
        return str((self.latitude,self.longitude))
    def __repr__(self):
        return str((self.latitude,self.longitude))
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# RoveyPosClass:
#    Creates a class for the location of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
class RoveyPosClass(object):

    def __init__(self, lat, lng, roll, pitch, yaw):
        self.latitude = lat
        self.longitude = lng  
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
          
    def setCoords(self, lat, lng):
        self.latitude = lat
        self.longitude = lng 

    def setOrientation(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    def __repr__(self):
        return str((self.latitude,self.longitude))  
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Vector2D:
#    Creates a class for a vector in 2d space
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
class Vector2D():
    def __init__(self, lat, lng):
        self.latitude = lat
        self.longitude = lng
    
    def get_length(self):
        return math.sqrt(self.latitude**2 + self.longitude**2)

    def normalise(self):
        length = self.get_length
        self.latitude = self.latitude/length
        self.longitude = self.longitude/length

    def scale(self, factor):
        self.latitude *= factor
        self.longitude *= factor

    @staticmethod
    def make_from_positions(from_pos, to_pos):
        lat_diff = from_pos.latitude - to_pos.latitude
        lng_diff = from_pos.longitude - to_pos.longitude
        return Vector2D(lat_diff, lng_diff)