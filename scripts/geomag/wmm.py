## A script for illustrating the use of the geomag script.
# Adapted from the geomagc software and World Magnetic Model of the NOAA
# Satellite and Information Service, National Geophysical Data Center
# http://www.ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml
import geomag 
## latitude and longitude of Hanksville Utah

if __name__ == "__main__":    
    latitude = 38.3730
    longitude = -110.7140
    alt = 1.3 #height above sea level
    dateOfComp = 2019.5 # Approx date of competition

    gm = geomag.GeoMag() ## Blank should be default

    magneticField = gm.GeoMag(latitude,longitude,alt)

    print(magneticField.dec)

# Usage of the different values returned in the
# magnetic field object are seen here:
# See the documentation for more info on coordinate frames

# retobj = RetObj()
# retobj.dec = dec  
# retobj.dip = dip 
# retobj.ti = ti
# retobj.bh = bh
# retobj.bx = bx
# retobj.by = by
# retobj.bz = bz
# retobj.lat = dlat
# retobj.lon = dlon
# retobj.alt = h
# retobj.time = time
