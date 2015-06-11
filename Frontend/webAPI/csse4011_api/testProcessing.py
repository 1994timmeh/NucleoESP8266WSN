import csse4011_api.processing as processing
import numpy as np
import matplotlib.pyplot as plt
import csv
import time
#from Pillow import Image
from .models import Node, Signal, CorrMax, Frame, VehicleEstimate

def velocityModel(t):
	return 100.0 + np.sin(t) #constant velocity

def simulation(deployment, node1, node2):
	start = processing.LatLon(-27.497977, 153.002238)
	end = processing.LatLon(-27.495143, 153.002754)

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
	frameNumHack = 0

	with open ('inputDataset.csv', 'wb') as csvfile:
		writer = csv.writer(csvfile, delimiter=',',quoting=csv.QUOTE_MINIMAL)

		while (distanceTraveled < distanceToTravel):
			velocity = velocityModel(t)
		
			carLocation = start.destinationPoint(distanceTraveled, bearing)
            
			node1delta = node1mic2.distanceTo(carLocation) - node1mic1.distanceTo(carLocation)
			node2delta = node2mic2.distanceTo(carLocation) - node2mic1.distanceTo(carLocation)
            
			print("node1Delts: " + str(inode1delta))
			print("node2Delts: " + str(node2delta))
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
				estimate = VehicleEstimate(latitude=res.raw.lat, longitude=res.raw.lon, latitudeFiltered = res.filtered.lat, longitudeFiltered = res.filtered.lon, Valid=res.valid, Type = "car", FrameNum = frameNumHack)
				estimate.save()
				frameNumHack += 1
				time.sleep(1);

			#writer.writerow([node1shift, 5,4,3,2,1, 0.0, velocity, 1.0, 1.0, 1.0, 1.0, n, 0])
			#writer.writerow([node2shift, 5,4,3,2,1, 0.0, velocity, 1.0, 1.0, 1.0, 1.0, n, 1])

			locations = np.concatenate((locations, [[carLocation.lon, carLocation.lat]]), axis=0)
			distanceTraveled = distanceTraveled + (velocity*deltaT)
			t = t + deltaT
			n = n + 1

	print("Time taken: " + str(t) + "s")
	

def runSimulation():
    nodes = Node.objects.all()
    node1 = processing.Node(float(nodes[0].latitude), float(nodes[0].longitude), int(nodes[0].Angle), 0.3, 48e3)
    node2 = processing.Node(float(nodes[1].latitude), float(nodes[1].longitude), int(nodes[1].Angle), 0.3, 48e3)

    deployment = processing.Deployment(node1, node2,  150)
    deployment.resetKalman(-27.497977, 153.002238, 14, 10)



    simulation(deployment, node1, node2)

