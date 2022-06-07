#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from dronekit import *
from pymavlink import mavutil
import time

iha = connect('/dev/ttyS0', wait_ready=True, baud=57600)

def anlik_konum_al_global(iha):
    return LocationGlobalRelative(iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon, iha.location.global_relative_frame.alt)

for i in range(0, 4):
    konum = anlik_konum_al_global(iha)
    print(konum)
    time.sleep(1)