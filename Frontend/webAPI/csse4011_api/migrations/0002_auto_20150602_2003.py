# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('csse4011_api', '0001_initial'),
    ]

    operations = [
        migrations.AlterField(
            model_name='node',
            name='longitude',
            field=models.CharField(max_length=21),
        ),
    ]
