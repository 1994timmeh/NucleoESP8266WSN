
# Latitude/Longititude distance and intersection formulas are adapted from 
# http://www.movable-type.co.uk/scripts/latlong-vectors.html#intersection

import numpy as np
import matplotlib.pyplot as plt

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


class Results:
	def __init__(self, valid, rawIntersection, filteredIntersection, category):
		self.valid = valid
		self.raw = rawIntersection
		self.filtered = filteredIntersection
		self.category = category

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

	def resetKalman(self, lat, lon, bearing, velocity):
		ndim = 4
		radEarth = 6371e3
		x = [lon, lat, velocity*np.sin(toRadians(bearing))/radEarth, velocity*np.cos(toRadians(bearing))/radEarth]
		cov_init=0.01*np.eye(ndim)
		measurementNoise = 0.0001
		processNoise = 0.0001

		self.Kalman = Kalman(x, cov_init, measurementNoise, processNoise, self.deltaT)

	
	def processFrame(self, featureVector1, featureVector2):
		
		# Change these when actual feature vector is implemented
		maxBin1 = featureVector1
		maxBin2 = featureVector2

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
		error1 = errorModel(angle1)
		#node1dist = self.node1.location.distanceTo(intersection)
		#worstCaseLocation1 = self.node1.location.destinationPoint(node1dist, angle1 )
		#localisationError1 = intersection.distanceTo(worstCaseLocation1)

		error2 = errorModel(angle2)
		#node2dist = self.node2.location.distanceTo(intersection)
		#worstCaseLocation2 = self.node2.location.destinationPoint(node2dist, angle2 )
		#localisationError2 = intersection.distanceTo(worstCaseLocation2)


		velocityLon = (intersection.lon - self.Kalman.x_hat[0]) / self.deltaT
		velocityLat = (intersection.lat - self.Kalman.x_hat[1]) / self.deltaT

		self.Kalman.update([intersection.lon, intersection.lat, velocityLon, velocityLat])

		filteredIntersection = LatLon(self.Kalman.x_hat[1], self.Kalman.x_hat[0])
		return Results(True, intersection, filteredIntersection, 'N/A')




def velocityModel(t):
	return 100.0 + np.sin(t) #constant velocity

def simulation(deployment, node1, node2):
	start = LatLon(-27.499158, 153.010547)
	end = LatLon(-27.500826, 153.008367)

	print "bearing: " + str(start.bearingTo(end)) + "degrees"
	print "distance: " + str(start.distanceTo(end)) + "m"

	bearing = start.bearingTo(end)
	distanceToTravel = start.distanceTo(end)

	locations = np.array([[start.lon, start.lat]])
	rawLocations = np.array([[start.lon, start.lat]])

	node1mic1 = node1.location.destinationPoint(0.15, (0.0 + toDegrees(node1.bearing)) % 360)
	node1mic2 = node1.location.destinationPoint(0.15, (180 + toDegrees(node1.bearing)) % 360)
	node2mic1 = node2.location.destinationPoint(0.15, (0.0 + toDegrees(node2.bearing)) % 360)
	node2mic2 = node2.location.destinationPoint(0.15, (180 + toDegrees(node2.bearing)) % 360)

	distanceTraveled = 0
	deltaT = 256/48e3
	t = 0.0
	while (distanceTraveled < distanceToTravel):
		velocity = velocityModel(t)
		
		carLocation = start.destinationPoint(distanceTraveled, bearing)

		node1delta = node1mic2.distanceTo(carLocation) - node1mic1.distanceTo(carLocation)
		node2delta = node2mic2.distanceTo(carLocation) - node2mic1.distanceTo(carLocation)

		node1shift = np.round(node1delta * node1.fs / node1.speedSound)
		node2shift = np.round(node2delta * node2.fs / node2.speedSound)

		res = deployment.processFrame(node1shift, node2shift)

		if (res.valid):
			rawLocations = np.concatenate((rawLocations, [[res.raw.lon, res.raw.lat]]), axis=0)

		locations = np.concatenate((locations, [[carLocation.lon, carLocation.lat]]), axis=0)
		distanceTraveled = distanceTraveled + (velocity*deltaT)
		t = t + deltaT

	print "Time taken: " + str(t) + "s"
	colours = np.random.rand(locations.shape[0])

	plt.figure()

	# Plot simulated trajectory
	plt.scatter(locations[:,0], locations[:,1],s=10, c='k', edgecolors='none', alpha=0.5, marker='o', label="true location")
	# Plot detected audio
	plt.scatter(rawLocations[:,0], rawLocations[:,1],s=10, c='g', edgecolors='none', alpha=0.5, marker='o', label="raw detected locations")

	# Plot node locations
	plt.scatter(node1.location.lon, node1.location.lat, s = 50, c='r', marker='x')
	plt.scatter(node2.location.lon, node2.location.lat, s = 50, c='r', marker='x')

	# Plot mic locations
	plt.scatter(node1mic1.lon, node1mic1.lat, s = 20, c='r', marker='+')
	plt.scatter(node1mic2.lon, node1mic2.lat, s = 20, c='r', marker='+')
	plt.scatter(node2mic1.lon, node2mic1.lat, s = 20, c='r', marker='+')
	plt.scatter(node2mic2.lon, node2mic2.lat, s = 20, c='r', marker='+')

	# Plott start and end points
	plt.scatter(start.lon, start.lat, s = 50, c='b', marker='x')
	plt.scatter(end.lon, end.lat, s = 50, c='b', marker='x')

	plt.axes().set_aspect('equal', adjustable='box')
	plt.show()




node1 = Node(-27.499543, 153.009910, 43, 0.3, 48e3)
node2 = Node(-27.499655, 153.010045, 43, 0.3, 48e3)

deployment = Deployment(node1, node2,  50)

deployment.resetKalman(-27.499543, 153.009910, 270, 10)

print node1.location

simulation(deployment, node1, node2)

quit()

if (res.valid):
	print "Node 1: " + node1.location.decimalPrint();
	print "Node 2: " + node2.location.decimalPrint();
	print "Raw Car: " +res.raw.decimalPrint()
	print "Filtered Car: " +res.filtered.decimalPrint()





