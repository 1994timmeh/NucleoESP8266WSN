# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('csse4011_api', '0005_signal'),
    ]

    operations = [
        migrations.AlterField(
            model_name='signal',
            name='Timestamp',
            field=models.BigIntegerField(),
        ),
    ]
