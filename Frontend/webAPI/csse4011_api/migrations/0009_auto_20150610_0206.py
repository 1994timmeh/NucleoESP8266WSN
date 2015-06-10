# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('csse4011_api', '0008_vehicleestimate'),
    ]

    operations = [
        migrations.CreateModel(
            name='Frame',
            fields=[
                ('id', models.AutoField(verbose_name='ID', primary_key=True, serialize=False, auto_created=True)),
                ('Node_ID', models.SmallIntegerField()),
                ('FrameNum', models.BigIntegerField()),
                ('MaxBin', models.BigIntegerField()),
                ('MaxFrequencies', models.CommaSeparatedIntegerField(max_length=10)),
                ('MaxValue', models.FloatField()),
                ('Mean', models.FloatField()),
                ('Power', models.FloatField()),
                ('Variance', models.FloatField()),
                ('Skew', models.FloatField()),
                ('Kurtosis', models.FloatField()),
            ],
        ),
        migrations.AddField(
            model_name='node',
            name='Angle',
            field=models.BigIntegerField(default=90),
            preserve_default=False,
        ),
    ]
