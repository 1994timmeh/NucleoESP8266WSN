from django.contrib import admin

from .models import Node, Signal

admin.site.register(Node)
admin.site.register(Signal)