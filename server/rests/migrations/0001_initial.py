# Generated by Django 4.2.16 on 2024-11-26 01:37

from django.db import migrations, models


class Migration(migrations.Migration):

    initial = True

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='ResponseLog',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('program', models.CharField(max_length=10)),
                ('conveyer_speed', models.CharField(max_length=10)),
                ('joint1', models.CharField(max_length=10)),
                ('joint2', models.CharField(max_length=10)),
                ('joint3', models.CharField(max_length=10)),
                ('joint4', models.CharField(max_length=10)),
                ('location_x', models.CharField(max_length=10)),
                ('location_y', models.CharField(max_length=10)),
                ('location_z', models.CharField(max_length=10)),
                ('obj_detection_result', models.CharField(max_length=20)),
                ('temp_result', models.CharField(max_length=10)),
                ('ultrasonic_result', models.CharField(max_length=10)),
                ('infrared_result', models.CharField(max_length=20)),
                ('pressure_result', models.CharField(max_length=10)),
                ('light_result', models.CharField(max_length=10)),
                ('cdate', models.DateTimeField(auto_now_add=True)),
            ],
        ),
    ]
