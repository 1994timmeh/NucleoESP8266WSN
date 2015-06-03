# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='Node',
            fields=[
                ('id', models.AutoField(serialize=False, auto_created=True, primary_key=True, verbose_name='ID')),
                ('Node_ID', models.SmallIntegerField()),
                ('latitude', models.CharField(max_length=20)),
                ('longitude', models.CharField(max_length=20)),
            ],
        ),
    ]
