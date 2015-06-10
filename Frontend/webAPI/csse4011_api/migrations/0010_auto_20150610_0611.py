# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('csse4011_api', '0009_auto_20150610_0206'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='vehicleestimate',
            name='Accuracy',
        ),
        migrations.AddField(
            model_name='vehicleestimate',
            name='Type',
            field=models.CharField(max_length=20, default='car'),
            preserve_default=False,
        ),
        migrations.AddField(
            model_name='vehicleestimate',
            name='Valid',
            field=models.SmallIntegerField(default=1),
            preserve_default=False,
        ),
        migrations.AddField(
            model_name='vehicleestimate',
            name='latitudeFiltered',
            field=models.CharField(max_length=21, default=1),
            preserve_default=False,
        ),
        migrations.AddField(
            model_name='vehicleestimate',
            name='longitudeFiltered',
            field=models.CharField(max_length=21, default=1),
            preserve_default=False,
        ),
    ]
