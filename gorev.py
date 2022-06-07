#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from vincenty import vincenty
from dronekit import *
from pymavlink import mavutil
import time

def velocity(velocity_x, velocity_y, iha):

    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    iha.send_mavlink(msg)

def goto_position_target_global_int(aLocation,iha):
    msg = iha.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        int(aLocation.lat*1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
        int(aLocation.lon*1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
        int(aLocation.alt), # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    iha.send_mavlink(msg)
    
def armOlveYuksel(yukseklik,iha):
    while iha.is_armable == False:
        print("Arm durumu sorgulaniyor...")
        time.sleep(1)
    iha.mode = "GUIDED"
    while iha.mode != "GUIDED":
        print("GUIDED moda gecis yapiliyor")
        time.sleep(1)
    print("GUIDED moda gecis yapildi")
    iha.armed=True
    while iha.armed == False:
        print("IHA arm oluyor")
        time.sleep(1)
    print("IHA arm oldu.\n")
    iha.simple_takeoff(yukseklik)
    while iha.location.global_relative_frame.alt < yukseklik*0.96:
        print("TAKEOFF : Su anki yukseklik =", iha.location.global_relative_frame.alt)
        time.sleep(1)
    print("TAKEOFF : SON yukseklik =", iha.location.global_relative_frame.alt)    
    print("TAKEOFF : Islem basariyla gerceklesti\n")

def anlik_konum_al(iha):
    return (iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon)

def anlik_konum_al_global(yukseklik,iha):
    return LocationGlobalRelative(iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon, yukseklik)

def gorev2(center_x, uzaklik, pid, vx, vy, hatax, hatay):
    connection = "127.0.0.1:14550"
    iha = connect(connection,wait_ready=True, timeout=100)
    
    home_global = anlik_konum_al_global(5, iha)
    home = anlik_konum_al(iha)
    
    konum1 = (-35.3628811, 149.1651609)
    konum2 = (-35.3636448, 149.1651607)
    havuz= (-35.3629598, 149.1651097)
    hat_baslangic = (-35.3629905, 149.1651088)
    hat_bitis = (-35.3635325, 149.1651095)
    
    konum1_global = LocationGlobalRelative(-35.3628811, 149.1651609, 5)
    konum2_global = LocationGlobalRelative(-35.3636448, 149.1651607, 5)
    havuz_global = LocationGlobalRelative(-35.3629598, 149.1651097, 5)
    hat_baslangic_global = LocationGlobalRelative(-35.3629905, 149.1651088, 5)
    hat_bitis_global = LocationGlobalRelative(-35.3635325, 149.1651095, 5)


    armOlveYuksel(5,iha)

    
    # KONUM 1'E GIDILIYOR | TUR 1
    while not 1000 * vincenty(anlik_konum_al(iha), konum1) < 1:
        goto_position_target_global_int(konum1_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(anlik_konum_al(iha), konum1)))
        print("HIZ = ", iha.groundspeed)
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(anlik_konum_al(iha), konum1)))
    

    # HAT BASLANGIC'A GIDILIYOR | TUR 1 
    while not 1000 * vincenty(anlik_konum_al(iha), hat_baslangic) < 0.3:
        goto_position_target_global_int(hat_baslangic_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(anlik_konum_al(iha), hat_baslangic)))
        print("HIZ = ", iha.groundspeed)    
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(anlik_konum_al(iha), hat_baslangic)))
    


    # HAT BITIS'E GIDILIYOR VE KIRMIZI ALAN BULUNUYOR | TUR 1
    sayac = 0
    kontrol = True
    while not 1000 * vincenty(anlik_konum_al(iha), hat_bitis) < 1:
        goto_position_target_global_int(hat_bitis_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(anlik_konum_al(iha), hat_bitis)))
        print("HIZ = ", iha.groundspeed)
        if center_x.value != 0 and kontrol:
            uzaklik.value = int(1000 * vincenty(anlik_konum_al(iha), hat_baslangic))
            print("------------------------UZAKLIK BULUNDU---------------------------\n", uzaklik.value)
            kirmizi_konum_global = iha.location.global_relative_frame
            print("----------------SU BIRAKMA ALANININ KOORDINATLARI:\n", anlik_konum_al(iha))
            kirmizi_konum = (kirmizi_konum_global.lat, kirmizi_konum_global.lon)
            sayac += 1
            if sayac > 1:
                kontrol = False
        time.sleep(0.5)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(anlik_konum_al(iha), hat_bitis)))

    
    # KONUM 2'E GIDILIYOR | TUR 1
    while not 1000 * vincenty(anlik_konum_al(iha), konum2) < 1:
        goto_position_target_global_int(konum2_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(anlik_konum_al(iha), konum2)))
        print("HIZ = ", iha.groundspeed)
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(anlik_konum_al(iha), konum2)))
    

    # BASLANGICA GIDILIYOR | TUR 1
    while not 1000 * vincenty(anlik_konum_al(iha), home) < 1:
        goto_position_target_global_int(home_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(anlik_konum_al(iha), home)))
        print("HIZ = ", iha.groundspeed)
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(anlik_konum_al(iha), home)))
    
    print("KIRMIZIYA UZAKLIK =", uzaklik.value, "metre") 
 
    
    # KONUM 1'E GIDILIYOR | TUR 2
    while not 1000 * vincenty(anlik_konum_al(iha), konum1) < 1:
        goto_position_target_global_int(konum1_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(anlik_konum_al(iha), konum1)))
        print("HIZ = ", iha.groundspeed)
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(anlik_konum_al(iha), konum1)))
    

    # HAVUZA GIDILIYOR | TUR 2
    while not 1000 * vincenty(anlik_konum_al(iha), havuz) < 0.3:
        goto_position_target_global_int(havuz_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(anlik_konum_al(iha), havuz)))
        print("HIZ = ", iha.groundspeed)    
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(anlik_konum_al(iha), havuz)))
    

    # HAVUZA INILIYOR VE SU ALINIYOR| TUR 2
    havuz_asagi = anlik_konum_al_global(1,iha)
    while not iha.location.global_relative_frame.alt < 1.2:
        goto_position_target_global_int(havuz_asagi,iha)
        print("\nAsagi iniliyor...\nHedefe uzaklik {:.2f}".format(iha.location.global_relative_frame.alt - 1))
        print("HIZ = ", iha.groundspeed)
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}" .format(iha.location.global_relative_frame.alt - 1))
    

    #SU ALMA
    

    # IHA YUKSELIYOR (HAVUZ) | TUR 2
    while not iha.location.global_relative_frame.alt > 4.8:
        goto_position_target_global_int(havuz_global,iha)
        print("\nYukari cikiliyor...\nHedefe uzaklik {:.2f}" .format(5 - iha.location.global_relative_frame.alt))
        print("HIZ = ", iha.groundspeed)    
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(5 - iha.location.global_relative_frame.alt))
    

    # HAT BASLANGIC'A GIDILIYOR | TUR 2
    while not 1000 * vincenty(anlik_konum_al(iha), hat_baslangic) < 0.3:
        goto_position_target_global_int(hat_baslangic_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(anlik_konum_al(iha), hat_baslangic)))
        print("HIZ = ", iha.groundspeed)    
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(anlik_konum_al(iha), hat_baslangic)))
    

    # KIRMIZIYA GIDILIYOR (SU BIRAKMA) | TUR 2
    while not 1000 * vincenty(anlik_konum_al(iha), kirmizi_konum) < 1:
        goto_position_target_global_int(kirmizi_konum_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" 
              .format(1000 * vincenty(anlik_konum_al(iha), kirmizi_konum)))
        print("HIZ = ", iha.groundspeed)
        time.sleep(0.5)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}" 
          .format(1000 * vincenty(anlik_konum_al(iha), kirmizi_konum)))
    

    # PID | TUR 2
    i = 6
    while 1:
        if center_x.value != 0:
            pid.value = 1
            print("PID calisiyor...........")
	    time.sleep(2)
	    while abs(hatay.value) > 20 or abs(hatax.value) > 20:
		print("ERROR x = {}\nERROR y = {}\nVX = {}\nVY = {}".format(hatax.value, hatax.value, vx.value, vy.value))
		velocity(vx.value, vy.value, iha)
		time.sleep(0.5)
            print("PID tamamlandı")
	    break
        else: 
            yeni_konum = anlik_konum_al_global(i, iha)
            goto_position_target_global_int(yeni_konum, iha)
            i += 1
	    time.sleep(1)
	    print("IHA'nin yerden yuksekligi:",iha.location.global_relative_frame.alt)

    print("IHA'nin yerden yuksekligi:",iha.location.global_relative_frame.alt)
    
    if iha.location.global_relative_frame.alt > 10:
        yeni_konum = anlik_konum_al_global(5, iha)
        goto_position_target_global_int(yeni_konum, iha)
	while iha.location.global_relative_frame.alt > 5.3: 
	    print("IHA'nin yerden yuksekligi:",iha.location.global_relative_frame.alt)
	    time.sleep(0.5)
        print("PID tekrar baslatiliyor.\nERROR x = {}\nERROR y = {}".format(hatax.value, hatay.value))  
        time.sleep(2) 
        while abs(hatay.value) > 20 or abs(hatax.value) > 20:
            print("ERROR x =", hatax.value)
            print("ERROR y =", hatay.value)
            print("VX =", vx.value)
            print("VY =", vy.value)	
            velocity(vx.value, vy.value, iha)
            time.sleep(0.5)	
	
    pid.value = 0

    
    # KIRMIZIYA INIS YAPILIYOR | TUR 2    
    kirmizi_asagi = anlik_konum_al_global(1,iha)
    while not iha.location.global_relative_frame.alt < 1.2:
        goto_position_target_global_int(kirmizi_asagi,iha)
        print("\nAsagi iniliyor...\nHedefe uzaklik {:.2f}".format(iha.location.global_relative_frame.alt - 1))
        print("HIZ = ", iha.groundspeed)
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}" .format(iha.location.global_relative_frame.alt - 1))
    

    #SU BOSALTMA
    

    # IHA YUKSELIYOR (KIRMIZI) | TUR 2 
    kirmizi_yukari = anlik_konum_al_global(5,iha)
    while not iha.location.global_relative_frame.alt > 4.8:
        goto_position_target_global_int(kirmizi_yukari,iha)
        print("\nYukari cikiliyor...\nHedefe uzaklik {:.2f}" .format(5 - iha.location.global_relative_frame.alt))
        print("HIZ = ", iha.groundspeed)    
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(5 - iha.location.global_relative_frame.alt))
    

    # HAT BITIS'E GIDILIYOR | TUR 2 
    while not 1000 * vincenty(anlik_konum_al(iha), hat_bitis) < 0.3:
        goto_position_target_global_int(hat_bitis_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(anlik_konum_al(iha), hat_bitis)))
        print("HIZ = ", iha.groundspeed)    
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(anlik_konum_al(iha), hat_bitis)))

    
    # KONUM 2'YE GIDILIYOR | TUR 2 
    while not 1000 * vincenty(anlik_konum_al(iha), konum2) < 1:
        goto_position_target_global_int(konum2_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(anlik_konum_al(iha), konum2)))
        print("HIZ = ", iha.groundspeed)
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(anlik_konum_al(iha), konum2)))
    

    # BASLANGICA GIDILIYOR | TUR 2
    while not 1000 * vincenty(anlik_konum_al(iha), home) < 1:
        goto_position_target_global_int(home_global,iha)
        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(anlik_konum_al(iha), home)))
        print("HIZ = ", iha.groundspeed)
        time.sleep(1)
    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(anlik_konum_al(iha), home)))
    
    iha.mode = "LAND"
