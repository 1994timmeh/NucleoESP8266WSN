from django.db import models


class Node(models.Model):
	Node_ID = models.SmallIntegerField();
	latitude = models.CharField(max_length=21);
	longitude = models.CharField(max_length=21);
	
class Signal(models.Model):
	Node_ID = models.SmallIntegerField();
	Angle = models.FloatField();
	Intensity = models.FloatField();
	Timestamp = models.BigIntegerField();


