import processing
import numpy as np
import matplotlib.pyplot as plt
import csv
#from Pillow import Image
from models import Node, Signal, CorrMax, Frame, VehicleEstimate

def velocityModel(t):
	return 100.0 + np.sin(t) #constant velocity

def simulation(deployment, node1, node2):
	start = processing.LatLon(-27.499158, 153.010547)
	end = processing.LatLon(-27.500826, 153.008367)

	print("bearing: " + str(start.bearingTo(end)) + "degrees")
	print("distance: " + str(start.distanceTo(end)) + "m")

	bearing = start.bearingTo(end)
	distanceToTravel = start.distanceTo(end)

	midpoint = node1.location.midpointTo(node2.location)

	locations = np.array([[start.lon, start.lat]])
	rawLocations = np.array([[start.lon, start.lat]])
	filteredLocations = np.array([[start.lon, start.lat]])

	rawTimeError = np.array([[0.0,0.0]])
	filteredTimeError = np.array([[0.0,0.0]])

	rawDistError = np.array([[0.0,0.0]])
	filteredDistError = np.array([[0.0,0.0]])

	node1mic1 = node1.location.destinationPoint(0.15, (0.0 + processing.toDegrees(node1.bearing)) % 360)
	node1mic2 = node1.location.destinationPoint(0.15, (180 + processing.toDegrees(node1.bearing)) % 360)
	node2mic1 = node2.location.destinationPoint(0.15, (0.0 + processing.toDegrees(node2.bearing)) % 360)
	node2mic2 = node2.location.destinationPoint(0.15, (180 + processing.toDegrees(node2.bearing)) % 360)

	distanceTraveled = 0
	deltaT = 256/48e3
	t = 0.0
	n = 0

	with open ('inputDataset.csv', 'wb') as csvfile:
		writer = csv.writer(csvfile, delimiter=',',quoting=csv.QUOTE_MINIMAL)

		while (distanceTraveled < distanceToTravel):
			velocity = velocityModel(t)
		
			carLocation = start.destinationPoint(distanceTraveled, bearing)

			node1delta = node1mic2.distanceTo(carLocation) - node1mic1.distanceTo(carLocation)
			node2delta = node2mic2.distanceTo(carLocation) - node2mic1.distanceTo(carLocation)

			node1shift = np.round(node1delta * node1.fs / node1.speedSound)
			node2shift = np.round(node2delta * node2.fs / node2.speedSound)

			node1measurement = processing.Measurement(int(node1shift), [0,0,0,0,0], 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
			node2measurement = processing.Measurement(int(node2shift), [0,0,0,0,0], 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)

			res = deployment.processFrame(node1measurement, node2measurement, 1)

			if (res.valid):
			    rawLocations = np.concatenate((rawLocations, [[res.raw.lon, res.raw.lat]]), axis=0)
			    filteredLocations = np.concatenate((filteredLocations, [[res.filtered.lon, res.filtered.lat]]), axis=0)
			    rawTimeError = np.concatenate((rawTimeError, [[t, carLocation.distanceTo(res.raw)]]), axis=0)
			    filteredTimeError = np.concatenate((filteredTimeError, [[t, carLocation.distanceTo(res.filtered)]]), axis=0)
			    rawDistError = np.concatenate((rawDistError, [[midpoint.distanceTo(res.raw), carLocation.distanceTo(res.raw)]]), axis=0)
			    filteredDistError = np.concatenate((filteredDistError, [[midpoint.distanceTo(res.filtered), carLocation.distanceTo(res.filtered)]]), axis=0)
                estimate = VehicleEstimate(latitude=res.raw.lat, longitude=res.raw.lon, latitudeFiltered = res.filtered.lat, longitudeFiltered = res.filtered.lon, Valid=res.valid, Type = "car", FrameNum=globalVal.val)
                estimate.save()


			#writer.writerow([node1shift, 5,4,3,2,1, 0.0, velocity, 1.0, 1.0, 1.0, 1.0, n, 0])
			#writer.writerow([node2shift, 5,4,3,2,1, 0.0, velocity, 1.0, 1.0, 1.0, 1.0, n, 1])

			locations = np.concatenate((locations, [[carLocation.lon, carLocation.lat]]), axis=0)
			distanceTraveled = distanceTraveled + (velocity*deltaT)
			t = t + deltaT
			n = n + 1

	print("Time taken: " + str(t) + "s")
	colours = np.random.rand(locations.shape[0])

	plt.figure()

	# Plot simulated trajectory
	plt.scatter(locations[:,0], locations[:,1],s=10, c='k', edgecolors='none', alpha=0.5, marker='o', label="true location")
	# Plot detected audio
	#plt.scatter(rawLocations[:,0], rawLocations[:,1],s=10, c='g', edgecolors='none', alpha=0.5, marker='o', label="raw detected locations")
	# Plot filtered locations
	#plt.scatter(filteredLocations[:,0], filteredLocations[:,1],s=10, c='r', edgecolors='none', alpha=0.5, marker='o', label="filtered locations")

	# Plot node locations
	plt.scatter(node1.location.lon, node1.location.lat, s = 50, c='r', marker='x')
	plt.scatter(node2.location.lon, node2.location.lat, s = 50, c='r', marker='x')

	# Plot mic locations
	plt.scatter(node1mic1.lon, node1mic1.lat, s = 20, c='r', marker='+')
	plt.scatter(node1mic2.lon, node1mic2.lat, s = 20, c='r', marker='+')
	plt.scatter(node2mic1.lon, node2mic1.lat, s = 20, c='r', marker='+')
	plt.scatter(node2mic2.lon, node2mic2.lat, s = 20, c='r', marker='+')

	# Plot start and end points
	plt.scatter(start.lon, start.lat, s = 50, c='b', marker='x')
	plt.scatter(end.lon, end.lat, s = 50, c='b', marker='x')

	# Lock 1:1 aspect ratio
	plt.axes().set_aspect('equal', adjustable='box')
	plt.xlim(end.lon - 0.0001, start.lon + 0.0001)
	plt.ylim(end.lat - 0.0001, start.lat + 0.0001)
	plt.title('Simulation Ground Truth')
	plt.show()
	# Plot node locations
	plt.scatter(node1.location.lon, node1.location.lat, s = 50, c='r', marker='x')
	plt.scatter(node2.location.lon, node2.location.lat, s = 50, c='r', marker='x')
	plt.scatter(start.lon, start.lat, s = 50, c='b', marker='x')
	plt.scatter(end.lon, end.lat, s = 50, c='b', marker='x')
	plt.axes().set_aspect('equal', adjustable='box')
	plt.scatter(rawLocations[:,0], rawLocations[:,1],s=10, c='g', edgecolors='none', alpha=0.5, marker='o', label="raw detected locations")
	plt.xlim(end.lon - 0.0001, start.lon + 0.0001)
	plt.ylim(end.lat - 0.0001, start.lat + 0.0001)
	plt.title('Simulation Raw Locations')
	plt.show()

	# Plot node locations
	plt.scatter(node1.location.lon, node1.location.lat, s = 50, c='r', marker='x')
	plt.scatter(node2.location.lon, node2.location.lat, s = 50, c='r', marker='x')
	plt.scatter(start.lon, start.lat, s = 50, c='b', marker='x')
	plt.scatter(end.lon, end.lat, s = 50, c='b', marker='x')
	plt.axes().set_aspect('equal', adjustable='box')
	plt.scatter(filteredLocations[:,0], filteredLocations[:,1],s=10, c='r', edgecolors='none', alpha=0.5, marker='o', label="filtered locations")
	plt.xlim(end.lon - 0.0001, start.lon + 0.0001)
	plt.ylim(end.lat - 0.0001, start.lat + 0.0001)
	plt.title('Simulation Filtered Locations')
	plt.show()

	plt.figure()
	plt.subplot(211)
	plt.title('Simulation: Error against Time')
	plt.xlabel('Time (s)')
	plt.ylabel('Error - Unfiltered (m)')
	plt.ylim(0, 20)
	plt.plot(rawTimeError[:,0], rawTimeError[:,1])
	plt.subplot(212)
	plt.xlabel('Time (s)')
	plt.ylabel('Error - Kalman Filtered (m)')
	plt.ylim(0, 20)
	plt.plot(filteredTimeError[:,0], filteredTimeError[:,1])
	plt.show()

	plt.figure()
	plt.subplot(211)
	plt.title('Simulation: Error against Distance to deployment midpoint')
	plt.xlabel('Distance (m)')
	plt.ylabel('Error - Unfiltered (m)')
	plt.scatter(rawDistError[:,0], rawDistError[:,1])
	plt.ylim(0, 20)
	plt.subplot(212)
	plt.xlabel('Distance (m)')
	plt.ylabel('Error - Kalman Filtered (m)')
	plt.scatter(filteredDistError[:,0], filteredDistError[:,1])
	plt.ylim(0, 20)
	plt.show()


nodes = Node.objects.all()
node1 = processing.Node(float(nodes[0].latitude), float(nodes[0].longitude), int(nodes[0].Angle), 0.3, 48e3)
node2 = processing.Node(float(nodes[1].latitude), float(nodes[1].longitude), int(nodes[1].Angle), 0.3, 48e3)

deployment = processing.Deployment(node1, node2,  150)
deployment.resetKalman(float(nodes[0].latitude), float(nodes[1].longitude), 0, 0)



simulation(deployment, node1, node2)
