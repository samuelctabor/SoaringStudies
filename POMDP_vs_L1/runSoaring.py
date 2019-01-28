#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, Command
import time
import math
import os
import sys

def readmission(aFileName):
    """
    Load a mission from a file into a list.

    This function is used by upload_mission().
    """
    print("Reading mission from file: %s\n" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist


def upload_mission(aFileName):
        """
        Upload a mission from a file.
        """
        #Read mission from file
        missionlist = readmission(aFileName)

        print("\nUpload mission from a file: %s" % aFileName)
        #Clear existing mission from vehicle
        print(' Clear mission')
        cmds = vehicle.commands
        cmds.clear()
        #Add new mission to vehicle
        for command in missionlist:
            cmds.add(command)
        print(' Upload mission')
        vehicle.commands.upload()

vehicle = connect('127.0.0.1:14551', wait_ready=True, rate=8)

vehicle.wait_ready('autopilot_version')

print("Basic pre-arm checks")

vehicle.parameters["SR0_EXTRA1"] = 8

print("Setting parameters ...")

for param in sys.argv:
    strings=param.split("=")
    if len(strings)==2:
        vehicle.parameters[strings[0]] = float(strings[1])

print("Uploading mission ...")
upload_mission('/home/samuel/Personal/ardupilot/Tools/autotest/ArduPlane-Missions/CMAC-soar.txt');

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

