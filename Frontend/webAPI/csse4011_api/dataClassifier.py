import numpy as np
import processing
import csv
import matplotlib.pyplot as plt

#from matplotlib import ion
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

try:

	node = processing.Node(0.0, 0.0, 0.0, 0.3, 48e3)

	inputFilename = 'inputDataset.csv'
	outputFilename = 'testDataset.csv'

	data = np.array([[0.0,0.0,0.0]])

	#Load dataset
	outputcsv = open(outputFilename, 'wb')
	writer = csv.writer(outputcsv, delimiter=',',quoting=csv.QUOTE_MINIMAL)
	with open(inputFilename, 'rb') as csvfile:
		reader = csv.reader(csvfile, delimiter=',')

		plt.figure()
		plt.ion()

		lastFrame = 0
		for line in reader:
				maxBin = float(line[0])
				maxFrequencies = np.zeros(5)
				maxFrequencies[0] = float(line[1])
				maxFrequencies[1] = float(line[2])
				maxFrequencies[2] = float(line[3])
				maxFrequencies[3] = float(line[4])
				maxFrequencies[4] = float(line[5])
				value = float(line[6])
				power = float(line[7])
				mean = float(line[8])
				variance = float(line[9])
				stdDev = np.sqrt(variance)
				skew = float(line[10])
				kurtosis = float(line[11])
			
				frameNumber = float(line[12])
				#node = int(float(line[13]))

				vector = [maxFrequencies[0],maxFrequencies[1],maxFrequencies[2],maxFrequencies[3],maxFrequencies[4], \
					power, mean, variance, stdDev, skew, kurtosis]

				for i in range(int(np.round(frameNumber - lastFrame - 1))):
					data = np.concatenate((data, [[lastFrame + i + 1, -10.0, 0.0]]), axis=0)

				angle = processing.toDegrees(node.getAngle(int(maxBin), False))
				data = np.concatenate((data, [[frameNumber, angle, power]]), axis=0)

				#Display data
				plt.subplot(211)
				plt.title('Angle')
				plt.xlabel('Frame Identifier')
				plt.ylabel('Angle (deg)')
				#plt.plot(data[:,0], data[:,1])

				cmap = ListedColormap(['r', 'k'])
				norm = BoundaryNorm([-1, 0.0, 180], cmap.N)

				points = np.array([data[:,0], data[:,1]]).T.reshape(-1, 1, 2)
				segments = np.concatenate([points[:-1], points[1:]], axis=1)
				lc = LineCollection(segments, cmap=cmap, norm=norm)
				lc.set_array(data[:,1])
				lc.set_linewidth(1)
				plt.gca().add_collection(lc)
				plt.xlim(data[:,0].min(), data[:,0].max())
				plt.ylim(-15, 180)

				plt.subplot(212)
				plt.xlabel('Frame Identifier')
				plt.ylabel('Acoustic Energy')

				cmap = ListedColormap(['r', 'k'])
				norm = BoundaryNorm([-1, 1.0, 180], cmap.N)

				points = np.array([data[:,0], data[:,2]]).T.reshape(-1, 1, 2)
				segments = np.concatenate([points[:-1], points[1:]], axis=1)
				lc = LineCollection(segments, cmap=cmap, norm=norm)
				lc.set_array(data[:,1])
				lc.set_linewidth(1)
				plt.gca().add_collection(lc)
				plt.xlim(data[:,0].min(), data[:,0].max())
				plt.ylim(0.0, data[:,2].max() + 1)

				#plt.show()

				#Get classification from user
				classifier = input("Angle: {0}, power{0} :".format(angle, power))
				dataVector = np.concatenate((vector, [int(classifier)]))
				writer.writerow(dataVector)
				lastFrame = frameNumber
	plt.show()
except KeyboardInterrupt:
	outputcsv.close()
	csvfile.close()
