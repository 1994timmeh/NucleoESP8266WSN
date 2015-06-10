from django.conf.urls import url

from . import views

urlpatterns = [
	url(r'^Nodes/$', views.Nodes, name='Nodes'),
	url(r'^Signals/(?P<timeStamp>[0-9]+)/$', views.Signals),
	url(r'^CorrMaxs/(?P<timeStamp>[0-9]+)/$', views.CorrMaxs),
	url(r'^VehicleEstimates/(?P<timeStamp>[0-9]+)/$', views.VehicleEstimates),
	url(r'^StartTcpClient/$', views.StartTcpClient, name='StartTcpClient'),
    url(r'^$', views.index, name='index'),
]
