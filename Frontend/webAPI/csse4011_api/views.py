from django.shortcuts import HttpResponse
from django.core import serializers

from .models import Node, Signal


def index(request):
    return HttpResponse("This is the index.")

def Nodes(request):
    output = serializers.serialize("json", Node.objects.all())
    return HttpResponse(output)
	
	
def Signals(request, timeStamp):
    output = serializers.serialize("json", Signal.objects.all())
    return HttpResponse(output)