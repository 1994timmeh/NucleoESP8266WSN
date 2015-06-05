#http://www.movable-type.co.uk/scripts/latlong-vectors.html#intersection
import numpy as np

class LatLon():


	def __init__(self, latitude, longitude):
		self.lat = latitude
		self.lon = longitude

	def toVector(self):
		result = [0.0,0.0,0.0]
		latitude = self.lat * np.pi / 180
		longitude = self.lon * np.pi / 180


		result[0] = np.cos(latitude)*np.cos(longitude)
		result[1] = np.cos(latitude)*np.sin(longitude)
		result[2] = np.sin(latitude)

		return result


def toLatLon(x,y,z):
	latitude = np.arctan2(z, np.sqrt(x*x + y*y))
	longitude = np.arctan2(y,x)
	return LatLon(latitude,longitude)


lat = -27.494059
lon = 153.005317

vector = LatLon(lat,lon).toVector()

data = toLatLon(vector[0], vector[1], vector[2])

print data.lat*180/np.pi, data.lon*180/np.pi





