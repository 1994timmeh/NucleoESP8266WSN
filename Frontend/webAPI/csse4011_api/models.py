from django.db import models


class Node(models.Model):
	Node_ID = models.SmallIntegerField();
	latitude = models.CharField(max_length=21);
	longitude = models.CharField(max_length=21);
	Angle = models.BigIntegerField();
	
class Signal(models.Model):
	Node_ID = models.SmallIntegerField();
	Angle = models.FloatField();
	Intensity = models.FloatField();
	Timestamp = models.BigIntegerField();
	
	
class CorrMax(models.Model):
	Node_ID = models.SmallIntegerField();
	MaxBin = models.FloatField();
	MaxValue = models.FloatField();
	Timestamp = models.BigIntegerField();
	
class VehicleEstimate(models.Model):
	latitude = models.CharField(max_length=21);
	longitude = models.CharField(max_length=21);
	Accuracy = models.FloatField();

class Frame(models.Model):
	Node_ID = models.SmallIntegerField();
	FrameNum = models.BigIntegerField();
	MaxBin = models.BigIntegerField();
	MaxFrequencies = models.CommaSeparatedIntegerField(max_length = 10);
	MaxValue = models.FloatField();
	Power =  models.FloatField();
	Mean =  models.FloatField();
	Power =  models.FloatField();
	Variance =  models.FloatField();
	Skew =  models.FloatField();
	Kurtosis =  models.FloatField();
	

