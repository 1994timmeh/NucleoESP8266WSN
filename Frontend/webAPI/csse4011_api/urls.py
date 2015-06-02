from django.conf.urls import url

from . import views

urlpatterns = [
	url(r'^Nodes/$', views.Nodes, name='Nodes'),
    url(r'^$', views.index, name='index'),
]
