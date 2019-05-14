import math, copy

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

    def __ne__(self, other):
        return not self.__eq__(other)

    def __eq__(self, other):
        k_tolerance = 1e-5 # 0.00001 lat lng dist ~= 1m
        try:
            return (
                self.latitude - other.latitude < k_tolerance 
                and self.longitude - other.longitude < k_tolerance)
        except:
            return False

    def shift(self, vector):
        self.latitude += vector.latitude
        self.longitude += vector.longitude

    @staticmethod
    def makeShiftedWaypoint(waypoint, vector):
        new_waypoint = copy.deepcopy(waypoint)
        new_waypoint.shift(vector)
        return new_waypoint

    def __str__(self):
        return str((self.latitude,self.longitude))
    def __repr__(self):
        return str((self.latitude,self.longitude))

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# RoveyPosClass:
#    Creates a class for the location of the rover
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
class RoveyPosClass(WaypointClass):

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

    def __eq__(self, other):
        try:
            return (
                WaypointClass.__eq__(self, other)        
                and self.yaw == other.yaw
                and self.pitch == other.pitch
                and self.roll == other.roll
                )
        except:
            return False

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
    
    def getLength(self):
        return math.sqrt(self.latitude**2 + self.longitude**2)

    def normalise(self):
        length = self.getLength()
        self.latitude = self.latitude/length
        self.longitude = self.longitude/length

    def scale(self, factor):
        self.latitude *= factor
        self.longitude *= factor

    def rotate(self, angle):
        angle = -angle # bearings rotate clockwise, radians rotate anticlockwise
        radians = math.radians(angle)
        new_latitude = self.latitude * math.cos(radians) - self.longitude * math.sin(radians)
        self.longitude = self.latitude * math.sin(radians) + self.longitude * math.cos(radians)
        self.latitude = new_latitude

    def invert(self):
        self.latitude = -self.latitude
        self.longitude = -self.longitude

    @staticmethod
    def makeFromPositions(from_pos, to_pos):
        lat_diff = from_pos.latitude - to_pos.latitude
        lng_diff = from_pos.longitude - to_pos.longitude
        return Vector2D(lat_diff, lng_diff)

    @staticmethod
    def makeFromBearingMagnitude(bearing, magnitude):
        lat = math.cos(bearing - 90) * magnitude
        lng = math.sin(bearing - 90) * magnitude
        return Vector2D(lat, lng)
