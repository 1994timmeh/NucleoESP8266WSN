
# Latitude/Longititude distance and intersection formulas are adapted from 
# http://www.movable-type.co.uk/scripts/latlong-vectors.html#intersection

import numpy as np

def toRadians(degree):
	return degree * np.pi / 180

def toDegrees(radian):
	return radian * 180 / np.pi

def ddToDms(dd):
   is_positive = dd >= 0
   dd = abs(dd)
   minutes,seconds = divmod(dd*3600,60)
   degrees,minutes = divmod(minutes,60)
   degrees = degrees if is_positive else -degrees
   return "{0}d,{1}m,{2}s".format(int(degrees), int(minutes), seconds)

def errorModel(angle):
	#Only need to model half the curve
	if (angle > 90):
		angle = 180 - angle
	return 15*np.exp(-0.1*angle)



class Vector3D():
	def __init__(self, x, y, z):
		self.coords = (x, y, z)

	def __repr__(self):
		return 'Vector3D%s' % (self.coords,)

	def x(self):
		return self.coords[0]

	def y(self):
		return self.coords[1]

	def z(self):
		return self.coords[2]

	def mag(self):
		return np.sqrt(self.x()**2 + self.y()**2 + self.z()**2)

	def __add__(self, other):
		return Vector3D(self.x() + other.x(), self.y() + other.y(), self.z() + other.z())

	def __sub__(self, other):
		return Vector3D(self.x() - other.x(), self.y() - other.y(), self.z() - other.z())

	def __mul__(self, other):
		return Vector3D(self.x()*other, self.y()*other, self.z()*other)

	def __div__(self, other):
		return Vector3D(self.x()/other, self.y()/other, self.z()/other)

	def dot(self, other):
		return self.x()*other.x()* + self.y()*other.y() + self.z()*other.z()

	def angle(self, other):
		sin = self.cross(other).mag()
		cos = self.dot(other)
		return np.arctan2(sin, cos)

	def unit(self):
		magnitude = self.mag()
		return Vector3D(self.x()/magnitude,self.y()/magnitude,self.z()/magnitude)

	def cross(self, other):
		crossX = (self.y()*other.z()) - (self.z()*other.y())
		crossY = (self.z()*other.x()) - (self.x()*other.z())
		crossZ = (self.x()*other.y()) - (self.y()*other.x())
		return Vector3D(crossX, crossY, crossZ)

	def toVector(self):
		return Vector3D(self.x(), self.y(), self.z())

	def toLatLon(self):
		latitude = np.arctan2(self.z(), np.sqrt((self.x()**2) + (self.y()**2)))
		longitude = np.arctan2(self.y(), self.x())
		return LatLon(toDegrees(latitude),toDegrees(longitude))


class LatLon():
	def __init__(self, latitude, longitude):
		self.lat = latitude
		self.lon = longitude

	def __repr__(self):
		return "Latitude: {0}; Longititude: {1}".format(ddToDms(self.lat), ddToDms(self.lon))

	def decimalPrint(self):
		return "Latitude: {0}; Longititude: {1}".format(self.lat, self.lon)

	def toVector(self):
		lat = toRadians(self.lat)
		lon = toRadians(self.lon)
		vectorX = np.cos(lat)*np.cos(lon)
		vectorY = np.cos(lat)*np.sin(lon)
		vectorZ = np.sin(lat)
		return Vector3D(vectorX, vectorY, vectorZ)

	def greatCircle(self, bearing):
		lat = toRadians(self.lat)
		lon = toRadians(self.lon)
		bearing = toRadians(bearing)

		vectorX = np.sin(lon)*np.cos(bearing) - np.sin(lat)*np.cos(lon)*np.sin(bearing)
		vectorY = -np.cos(lon)*np.cos(bearing) - np.sin(lat)*np.sin(lon)*np.sin(bearing)
		vectorZ = np.cos(lat)*np.sin(bearing)
		return Vector3D(vectorX, vectorY, vectorZ)

	def destinationPoint(self, distance, bearing):
		radius = 6371e3; #m

		angularDist = distance / radius
		c = self.greatCircle(bearing)
		p1 = self.toVector()
		x  = (p1 * np.cos(angularDist))
		y = (c.cross(p1) * np.sin(angularDist))
		p2 = (x + y).unit()
		return p2.toLatLon()

	def distanceTo(self, other):
		radius = 6371e3; #m
		p1 = self.toVector()
		p2 = other.toVector()

		delta = p1.angle(p2)
		return delta*radius


	def midpointTo(self, other):
		p1 = self.toVector()
		p2 = other.toVector()

		return (p1 + p2).unit().toLatLon()


	def intersection(self, selfBearing, other, otherBearing):
		point1 = self.toVector()
		point2 = other.toVector()

		bearing1 = self.greatCircle(selfBearing)
		bearing2 = other.greatCircle(otherBearing)

		int1 = bearing1.cross(bearing2)
		int2 = bearing2.cross(bearing1)

		midpoint = self.midpointTo(other)

		# Return the closest intersection with respect to midpoint
		# Probably only valid for distance between nodes much less
		# than radius of earth, as they are in this case
		dist1 = midpoint.distanceTo(int1)
		dist2 = midpoint.distanceTo(int2)

		if (dist1 < dist2):
			return int1.toLatLon()
		else:
			return int2.toLatLon()

class Node():
	def __init__(self, latitude, longitude, angularBearing, distance, samplingFrequency):
		self.speedSound = 340.29  #m/s at sea level
		self.location = LatLon(latitude, longitude)
		self.bearing = angularBearing
		self.micDist = distance
		self.micDistSq = self.micDist ** 2
		self.fs = samplingFrequency

	def getAngle(self, sampleOffset):
		distanceDifference = sampleOffset * (self.speedSound / self.fs)
		distanceDiffSq = distanceDifference ** 2
		
		arrivalAngle = np.arctan2(np.sqrt(np.abs(self.micDistSq - distanceDiffSq)), distanceDifference)

		return self.bearing + toDegrees(arrivalAngle)

class Deployment():

	def __init__(self, node1, node2, detectionRadius):
		self.node1 = node1
		self.node2 = node2
		self.radius = detectionRadius
		self.midpoint = self.node1.location.midpointTo(self.node2.location)

	
	def processFrame(self, featureVector1, featureVector2):
		
		# Change these when actual feature vector is implemented
		maxBin1 = featureVector1
		maxBin2 = featureVector2

		# Get source angle with reference to True North
		angle1 = self.node1.getAngle(maxBin1)
		angle2 = self.node2.getAngle(maxBin2)

		# Get intersection of angles
		intersection = self.node1.location.intersection(angle1, self.node2.location, angle2)

		# Determine distance from center of nodes to intersection point
		sourceDistance = self.midpoint.distanceTo(intersection)

		print "Average distance to source is: {0}m".format(sourceDistance)

		# Check that distance falls within acceptable region
		if (sourceDistance > self.radius):
			return 0

		# Get localisation error for each measured angle
		# Ignore for now due to inaccuracies, most likely from earth radius value
		error1 = errorModel(angle1)
		node1dist = self.node1.location.distanceTo(intersection)
		worstCaseLocation1 = self.node1.location.destinationPoint(node1dist, angle1 )
		localisationError1 = intersection.distanceTo(worstCaseLocation1)

		error2 = errorModel(angle2)
		node2dist = self.node2.location.distanceTo(intersection)
		worstCaseLocation2 = self.node2.location.destinationPoint(node2dist, angle2 )
		localisationError2 = intersection.distanceTo(worstCaseLocation2)

		return 1, intersection




node1 = Node(-27.499543, 153.009910, 0.0, 0.3, 48e3)
node2 = Node(-27.499655, 153.010045, 0.0, 0.3, 48e3)

deployment = Deployment(node1, node2,  50)

valid, location = deployment.processFrame(-15, 28)

if (valid):
	print "Node 1: " + node1.location.decimalPrint();
	print "Node 2: " + node2.location.decimalPrint();
	print "Car: " +location.decimalPrint()






