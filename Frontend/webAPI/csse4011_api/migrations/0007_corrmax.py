# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('csse4011_api', '0006_auto_20150603_2113'),
    ]

    operations = [
        migrations.CreateModel(
            name='CorrMax',
            fields=[
                ('id', models.AutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('Node_ID', models.SmallIntegerField()),
                ('MaxBin', models.FloatField()),
                ('MaxValue', models.FloatField()),
                ('Timestamp', models.BigIntegerField()),
            ],
        ),
    ]
