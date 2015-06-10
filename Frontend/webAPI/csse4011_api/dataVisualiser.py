import numpy as np
import processing
import csv
import matplotlib.pyplot as plt

from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

node = processing.Node(0.0, 0.0, 0.0, 0.3, 48e3)

datasetFilename = 'inputDataset.csv'

data1 = np.array([[0.0,0.0,0.0]])
data2 = np.array([[0.0,0.0,0.0]])

#Load dataset
with open(datasetFilename, 'rb') as csvfile:
	reader = csv.reader(csvfile, delimiter=',')

	lastFrame1 = 0
	lastFrame2 = 0
	for line in reader:
			maxBin = float(line[0])
			maxFrequencies = np.zeros(5)
			maxFrequencies[0] = float(line[1])
			maxFrequencies[1] = float(line[2])
			maxFrequencies[2] = float(line[3])
			maxFrequencies[3] = float(line[4])
			maxFrequencies[4] = float(line[5])
			power = float(line[7])
			mean = float(line[8])
			variance = float(line[9])
			stdDev = np.sqrt(variance)
			skew = float(line[10])
			kurtosis = float(line[11])
			frameNumber = float(line[12])

			nodeId = int(float(line[13]))

			if (nodeId == 0):
				for i in range(int(np.round(frameNumber - lastFrame1 - 1))):
					data1 = np.concatenate((data1, [[lastFrame1 + i + 1, -10.0, 0.0]]), axis=0)

				angle = processing.toDegrees(node.getAngle(int(maxBin), False))
				data1 = np.concatenate((data1, [[frameNumber, angle, power]]), axis=0)
				lastFrame1 = frameNumber
			if (nodeId == 1):
				for i in range(int(np.round(frameNumber - lastFrame2 - 1))):
					data2 = np.concatenate((data2, [[lastFrame2 + i + 1, -10.0, 0.0]]), axis=0)

				angle = processing.toDegrees(node.getAngle(int(maxBin), False))
				data2 = np.concatenate((data2, [[frameNumber, angle, power]]), axis=0)
				lastFrame2 = frameNumber

#Pad results if required
if (lastFrame1 > lastFrame2):
	for i in range(lastFrame1 - lastFrame2):
		data2 = np.concatenate((data2, [[lastFrame2 + i + 1, -10.0, 0.0]]), axis=0)
if (lastFrame2 > lastFrame1):
	for i in range(lastFrame2 - lastFrame1):
		data1 = np.concatenate((data1, [[lastFrame1 + i + 1, -10.0, 0.0]]), axis=0)

#Display data

plt.figure()
plt.subplot(311)
plt.title('Node 1 Angle')
plt.xlabel('Frame Identifier')
plt.ylabel('Angle (deg)')
plt.plot(data1[:,0], data1[:,1])
plt.xlim(data1[:,0].min(), data1[:,0].max())
plt.ylim(-15, 180)

plt.subplot(312)
plt.title('Node 2 Angle')
plt.xlabel('Frame Identifier')
plt.ylabel('Angle (deg)')
plt.plot(data2[:,0], data2[:,1])
plt.xlim(data2[:,0].min(), data2[:,0].max())
plt.ylim(-15, 180)

plt.subplot(313)
plt.xlabel('Frame Identifier')
plt.ylabel('Acoustic Energy')
avgEnergy = (data1[:,2] + data2[:,2])/2
plt.plot(data1[:,0], avgEnergy)
plt.xlim(data1[:,0].min(), avgEnergy.max())
plt.ylim(0.0, data1[:,2].max() + 1)

plt.show()
