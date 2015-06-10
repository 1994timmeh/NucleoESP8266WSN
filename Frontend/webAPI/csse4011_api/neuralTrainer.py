import numpy as np
import pybrain
import csv

from pybrain.tools.shortcuts import buildNetwork
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.tools.validation import Validator
from pybrain.tools.customxml import NetworkWriter

datasetFilename = 'testDataset.csv'
networkFilename = 'testNetwork.xml'
numInputs = 11
numOutputs = 1

outputCategories = 2

net = buildNetwork(numInputs, 4, numOutputs)

ds = SupervisedDataSet(numInputs, numOutputs)

#Load dataset
with open(datasetFilename, 'rb') as csvfile:
	reader = csv.reader(csvfile, delimiter=',')

	for line in reader:
		inputs = line[:numInputs]
		outputs = line[numInputs]
		ds.addSample(inputs, outputs)

#Train network
trainer = BackpropTrainer(net, ds)
trainer.trainUntilConvergence()

#Save network
NetworkWriter.writeToFile(net, networkFilename)

#Validate network
validation = np.zeros((outputCategories, outputCategories))
classificiationErrors = 0
for inputData, expectedOutput in ds:
	expectedOutput = int(np.round(expectedOutput))
	output = int(np.round(net.activate(inputData)))

	if (output >= outputCategories) or (output < 0):
		classificiationErrors = classificiationErrors + 1
	else:
		validation[expectedOutput][output] = validation[expectedOutput][output] + 1


print validation
print "Errors: " + str(classificiationErrors)

quit()


for inputData, expectedOutput in ds:
	output = net.activate(inputData)
	if output != expectedOutput:
		print str(inputData) + " did not match expected output " + str(output) + " " + str(expectedOutput)
