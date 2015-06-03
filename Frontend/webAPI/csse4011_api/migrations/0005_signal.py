# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('csse4011_api', '0004_auto_20150602_2011'),
    ]

    operations = [
        migrations.CreateModel(
            name='Signal',
            fields=[
                ('id', models.AutoField(verbose_name='ID', primary_key=True, serialize=False, auto_created=True)),
                ('Node_ID', models.SmallIntegerField()),
                ('Angle', models.FloatField()),
                ('Intensity', models.FloatField()),
                ('Timestamp', models.TimeField()),
            ],
        ),
    ]
