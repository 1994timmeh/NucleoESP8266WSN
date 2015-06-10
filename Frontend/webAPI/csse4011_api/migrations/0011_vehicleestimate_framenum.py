# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('csse4011_api', '0010_auto_20150610_0611'),
    ]

    operations = [
        migrations.AddField(
            model_name='vehicleestimate',
            name='FrameNum',
            field=models.BigIntegerField(default=0),
            preserve_default=False,
        ),
    ]
