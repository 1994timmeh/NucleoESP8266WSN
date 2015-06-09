
# Latitude/Longititude distance and intersection formulas are adapted from 
# http://www.movable-type.co.uk/scripts/latlong-vectors.html#intersection
import numpy as np
import csv
import pybrain
from pybrain.tools.customxml import NetworkReader

neuralNetworkFilename = 'testNetwork.xml'

def toRadians(degree):
	return np.float64(degree * np.pi / 180)

def toDegrees(radian):
	return np.float64(radian * 180 / np.pi)

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

def categoryToLabel(category):
	if (category == 0):
		return 'Car'
	else:
		return 'Not Car'

def generateFeatureVector(measurement):
	vector = np.zeros(11)
	vector[0] = float(measurement.frequencies[0])
	vector[1] = float(measurement.frequencies[1])
	vector[2] = float(measurement.frequencies[2])
	vector[3] = float(measurement.frequencies[3])
	vector[4] = float(measurement.frequencies[4])
	vector[5] = float(measurement.power)
	vector[6] = float(measurement.mean)
	vector[7] = float(measurement.variance)
	vector[8] = float(measurement.stdDev)
	vector[9] = float(measurement.skew)
	vector[10] = float(measurement.kurtosis)
	return vector

class Measurement():
	def __init__(self, maxBin, maxFrequencies, maxValue, power, mean, variance, skew, kurtosis):
		self.bin = maxBin
		self.value = maxValue
		self.frequencies = maxFrequencies
		self.power = power
		self.mean = mean
		self.variance = variance
		self.stdDev = np.sqrt(variance)
		self.skew = skew
		self.kurtosis = kurtosis

class Results:
	def __init__(self, valid, rawIntersection, filteredIntersection, category):
		self.valid = valid
		self.raw = rawIntersection
		self.filtered = filteredIntersection
		self.category = category

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

	def angleTo(self, other, sign):
		sin = self.cross(other).mag()
		cos = self.dot(other)
		if self.cross(other).dot(sign) < 0:
			sin = -sin
		else:
			sin = sin
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

	def bearingTo(self, other):
		lat1 = toRadians(self.lat)
		lon1 = toRadians(self.lon)
		lat2 = toRadians(other.lat)
		lon2 = toRadians(other.lon)

		dLon = lon2 - lon1
		dPhi = np.log(np.tan(lat2/2.0 + np.pi/4.0)/np.tan(lat1/2.0 + np.pi/4.0))
		if np.abs(dPhi) > np.pi:
			if dLon > 0.0:
				dLon = -(2.0*np.pi - dLon)
			else:
				dLon = -(2.0*np.pi + dLon)

		return (toDegrees(np.arctan2(dLon, dPhi)) + 360) % 360

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
		lat1 = toRadians(self.lat)
		lon1 = toRadians(self.lon)
		lat2 = toRadians(other.lat)
		lon2 = toRadians(other.lon)

		radius = 6371e3; #m

		d  = 2*np.arcsin(np.sqrt((np.sin((lat1-lat2)/2.0)**2 + np.cos(lat1)*np.cos(lat2)*(np.sin((lon1-lon2)/2.0))**2)))

		return d*radius


	def midpointTo(self, other):
		p1 = self.toVector()
		p2 = other.toVector()

		return (p1 + p2).unit().toLatLon()


	def intersection(self, selfBearing, other, otherBearing):
		lat1 = toRadians(self.lat)
		lon1 = toRadians(self.lon)
		lat2 = toRadians(other.lat)
		lon2 = toRadians(other.lon)

		theta13 = toRadians(selfBearing)
		theta23 = toRadians(otherBearing)

		deltaLat = lat2 - lat1
		deltaLon = lon2 - lon1

		delta12 = 2*np.arcsin(np.sqrt(np.sin(deltaLat/2)**2 + np.cos(lat1)*np.cos(lat2)*np.sin(deltaLon/2)**2))
		thetaA = np.arccos((np.sin(lat2) - np.sin(lat1)*np.cos(delta12))/(np.sin(delta12)*np.cos(lat1)))
		thetaB = np.arccos((np.sin(lat1) - np.sin(lat2)*np.cos(delta12))/(np.sin(delta12)*np.cos(lat2)))

		if (np.sin(lon2 - lon1) > 0):
			theta12 = thetaA
			theta21 = 2*np.pi - thetaB
		else:
			theta12 = 2*np.pi - thetaA
			theta21 = thetaB

		alpha1 = ((theta13 - theta12 + np.pi) % (2*np.pi)) - np.pi
		alpha2 = ((theta21 - theta23 + np.pi) % (2*np.pi)) - np.pi

		alpha3 = np.arccos(-np.cos(alpha1)*np.cos(alpha2) + np.sin(alpha1)*np.sin(alpha2)*np.cos(delta12))
		delta13 = np.arctan2(np.sin(delta12)*np.sin(alpha1)*np.sin(alpha2), np.cos(alpha2) + np.cos(alpha1)*np.cos(alpha3))
	
		lat3 = np.arcsin(np.sin(lat1)*np.cos(delta13) + np.cos(lat1)*np.sin(delta13)*np.cos(theta13))

		deltaLon13 = np.arctan2(np.sin(theta13)*np.sin(delta13)*np.cos(lat1), np.cos(delta13) - np.sin(lat1)*np.sin(lat3))
	
		lon3 = ((lon1 + deltaLon13 + np.pi) % (2*np.pi)) - np.pi

		#print "Debug"
		#print theta13, theta23
		#print theta12, theta21
		#print thetaA, thetaB
		#print delta12, delta13
		#print alpha1, alpha2, alpha3
		#print lat3, lon3
		#print "End Debug"

		return LatLon(toDegrees(lat3), toDegrees(lon3))

class Kalman:
    def __init__(self, x_init, cov_init, meas_err, proc_err, dt):
        self.ndim = len(x_init)
        self.A = np.array([(1, 0, dt, 0), (0, 1, 0, dt), (0, 0, 1, 0), (0, 0, 0, 1)]);
        self.H = np.array([(1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)])
        self.x_hat =  x_init
        self.cov = cov_init
        self.Q_k = np.eye(self.ndim)*proc_err
        self.R = np.eye(len(self.H))*meas_err

    def update(self, obs):

        # Make prediction
        self.x_hat_est = np.dot(self.A,self.x_hat)
        self.cov_est = np.dot(self.A,np.dot(self.cov,np.transpose(self.A))) + self.Q_k

        # Update estimate
        self.error_x = obs - np.dot(self.H,self.x_hat_est)
        self.error_cov = np.dot(self.H,np.dot(self.cov_est,np.transpose(self.H))) + self.R
        self.K = np.dot(np.dot(self.cov_est,np.transpose(self.H)),np.linalg.inv(self.error_cov))
        self.x_hat = self.x_hat_est + np.dot(self.K,self.error_x)
        if self.ndim>1:
            self.cov = np.dot((np.eye(self.ndim) - np.dot(self.K,self.H)),self.cov_est)
        else:
            self.cov = (1-self.K)*self.cov_est


class Node():
	def __init__(self, latitude, longitude, angularBearing, distance, samplingFrequency):
		self.speedSound = 340.29  #m/s at sea level
		self.location = LatLon(latitude, longitude)
		self.bearing = toRadians(angularBearing)
		self.micDist = distance
		self.micDistSq = self.micDist ** 2
		self.fs = samplingFrequency

	def getAngle(self, sampleOffset, invert):
		distanceDifference = sampleOffset * (self.speedSound / self.fs)
		distanceDiffSq = distanceDifference ** 2
		
		arrivalAngle = np.float64(np.arctan2(np.sqrt(np.abs(self.micDistSq - distanceDiffSq)), distanceDifference))
		if (invert):
			return (self.bearing - arrivalAngle) % (2*np.pi)
		else:
			return (self.bearing + arrivalAngle) % (2*np.pi)

class Deployment():

	def __init__(self, node1, node2, detectionRadius):
		self.framelength = 256 #samples
		self.deltaT = np.float64(self.framelength / 48e3)
		self.node1 = node1
		self.node2 = node2
		self.radius = detectionRadius
		self.midpoint = self.node1.location.midpointTo(self.node2.location)
		self.kalman = Kalman([0.0], [0.0], 0.0, 0.0, self.deltaT)
		self.classifier = NetworkReader.readFrom(neuralNetworkFilename)

	def resetKalman(self, lat, lon, bearing, velocity):
		ndim = 4
		radEarth = 6371e3
		x = [lon, lat, velocity*np.sin(toRadians(bearing))/radEarth, velocity*np.cos(toRadians(bearing))/radEarth]
		cov_init=0.1*np.eye(ndim)
		measurementNoise = 1
		processNoise = 0.00001

		self.Kalman = Kalman(x, cov_init, measurementNoise, processNoise, self.deltaT)

	
	def processFrame(self, node1Measurement, node2Measurement):
		
		# Change these when actual feature vector is implemented
		maxBin1 = node1Measurement.bin
		maxBin2 = node2Measurement.bin

		# Get source angle with reference to True North
		angle1 = self.node1.getAngle(maxBin1, False)
		angle2 = self.node2.getAngle(maxBin2, True)

		# Get intersection of angles
		intersection = self.node1.location.intersection(toDegrees(angle1), self.node2.location, toDegrees(angle2))

		#print maxBin1, toDegrees(angle1), maxBin2, toDegrees(angle2)
		#print intersection

		# Determine distance from center of nodes to intersection point
		sourceDistance = self.midpoint.distanceTo(intersection)

		#print "Average distance to source is: {0}m".format(sourceDistance)

		#Check that distance falls within acceptable region
		if (sourceDistance > self.radius):
			return Results(False, 0, 0, 0)

		# Get localisation error for each measured angle
		# Ignore for now due to inaccuracies, most likely from earth radius value
		#error1 = errorModel(angle1)
		#node1dist = self.node1.location.distanceTo(intersection)
		#worstCaseLocation1 = self.node1.location.destinationPoint(node1dist, angle1 )
		#localisationError1 = intersection.distanceTo(worstCaseLocation1)

		#error2 = errorModel(angle2)
		#node2dist = self.node2.location.distanceTo(intersection)
		#worstCaseLocation2 = self.node2.location.destinationPoint(node2dist, angle2 )
		#localisationError2 = intersection.distanceTo(worstCaseLocation2)

		velocityLon = (intersection.lon - self.Kalman.x_hat[0]) / self.deltaT
		velocityLat = (intersection.lat - self.Kalman.x_hat[1]) / self.deltaT

		self.Kalman.update([intersection.lon, intersection.lat, velocityLon, velocityLat])

		node1classification = self.classifier.activate(generateFeatureVector(node1Measurement))
		node2classification = self.classifier.activate(generateFeatureVector(node2Measurement))
		
		filteredIntersection = LatLon(self.Kalman.x_hat[1], self.Kalman.x_hat[0])

		if (node1classification != node2classification):
			return Results(True, intersection, filteredIntersection, 'No Car')
		else:
			return Results(True, intersection, filteredIntersection, categoryToLabel(node1classification))



