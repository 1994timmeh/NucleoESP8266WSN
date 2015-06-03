from django.shortcuts import HttpResponse
from django.core import serializers

from .models import Node


def index(request):
    return HttpResponse("This is the index.")

def Nodes(request):
    output = serializers.serialize("json", Node.objects.all())
    return HttpResponse(output)