import numpy as np
import processing
import csv
import matplotlib.pyplot as plt

node = processing.Node(0.0, 0.0, 0.0, 0.3, 48e3)


datasetFilename = 'inputDataset.csv'

data = np.array([[0.0,0.0,0.0]])

#Load dataset
with open(datasetFilename, 'rb') as csvfile:
	reader = csv.reader(csvfile, delimiter=',')

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


			angle = processing.toDegrees(node.getAngle(int(maxBin), False))
			data = np.concatenate((data, [[frameNumber, angle, power]]), axis=0)

#Display data
print data[0]
print data[100]


plt.figure()
plt.subplot(211)
plt.title('Angle')
plt.xlabel('Frame Identifier')
plt.ylabel('Angle (deg)')
plt.plot(data[:,0], data[:,1])
plt.subplot(212)
plt.xlabel('Frame Identifier')
plt.ylabel('Acoustic Energy')
plt.plot(data[:,0], data[:,2])
plt.show()
