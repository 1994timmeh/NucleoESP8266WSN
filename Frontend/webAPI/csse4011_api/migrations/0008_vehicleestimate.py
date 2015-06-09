# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('csse4011_api', '0007_corrmax'),
    ]

    operations = [
        migrations.CreateModel(
            name='VehicleEstimate',
            fields=[
                ('id', models.AutoField(serialize=False, auto_created=True, verbose_name='ID', primary_key=True)),
                ('latitude', models.CharField(max_length=21)),
                ('longitude', models.CharField(max_length=21)),
                ('Accuracy', models.FloatField()),
            ],
        ),
    ]
