#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal
import time
import math
import os
import sys

vehicle = connect('127.0.0.1:14551', wait_ready=True, rate=8)

vehicle.wait_ready('autopilot_version')

print("Basic pre-arm checks")

vehicle.parameters["SR0_EXTRA1"] = 8

print("Setting parameters ...")

for param in sys.argv:
    strings=param.split("=")
    if len(strings)==2:
        vehicle.parameters[strings[0]] = float(strings[1])


print("\nSet Vehicle.armed=True (currently: %s)" % vehicle.armed)
vehicle.armed = True


if not vehicle.armed:
    print(" Waiting for arm")

while not vehicle.armed:
    time.sleep(1)
    vehicle.armed = True
    sys.stdout.write('.')
    sys.stdout.flush()

vehicle.mode = VehicleMode("AUTO")

print("Waiting for thermalling")
while not vehicle.mode==VehicleMode("LOITER"):
    sys.stdout.write('.')
    sys.stdout.flush()
    time.sleep(1)
    
print("Waiting for cruise . . . ")
while not vehicle.mode==VehicleMode("AUTO"):
    sys.stdout.write('.')
    sys.stdout.flush()
    time.sleep(1)
    
time.sleep(2)

#Close vehicle object before exiting script
print("\nClose vehicle object")
vehicle.close()

# Kill the SITL instance
os.system("killall -9 arduplane")
os.system("killall -9 mavproxy.py")

print("Completed")

