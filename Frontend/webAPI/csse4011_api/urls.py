from django.conf.urls import url

from . import views

urlpatterns = [
	url(r'^Nodes/$', views.Nodes, name='Nodes'),
	url(r'^Signals/(?P<timeStamp>[0-9]+)/$', views.Signals),
    url(r'^$', views.index, name='index'),
]
