from django.contrib import admin

from .models import Node, Signal, VehicleEstimate

admin.site.register(Node)
admin.site.register(Signal)
admin.site.register(VehicleEstimate)