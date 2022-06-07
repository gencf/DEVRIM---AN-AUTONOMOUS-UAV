from vincenty import vincenty
from dronekit import *
from pymavlink import mavutil
import time
import math
connection = "127.0.0.1:14550"
iha = connect(connection,wait_ready=True, timeout=100)

## KOORDINATLAR 
direk1 = (-35.3636199, 149.165267)
direk2 = (-35.3629013, 149.1652678)
direk3 = (-35.363261, 149.164989)

def armOlveYuksel(yukseklik):
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

def calculate_initial_compass_bearing(pointA, pointB):
    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Sadece tuple arguman olarak gecerlidir")
    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])
    diffLong = math.radians(pointB[1] - pointA[1])
    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
            * math.cos(lat2) * math.cos(diffLong))
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

def gercek_kuzey(x, y, aci):
    aci = math.radians(aci)
    return x * math.cos(aci) + y * math.cos(math.radians(90) + aci)
    
def gercek_dogu(x, y, aci):
    aci = math.radians(aci)
    return x * math.sin(aci) + y * math.sin(math.radians(90) + aci)

def anlik_konum_al():
    return (iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon)

def cember(baslangic_aci, bitis_aci, yon, yaricap, x, y, z, direk_konumu):
    i = baslangic_aci
    aralik = 30
    zaman = 0.5
    while (yon == 1 and i < bitis_aci) or (yon == -1 and i > bitis_aci):
         position(x + yaricap*math.cos(math.radians(i)),(y + yaricap*math.sin(math.radians(i))), -z , iha, bearing)
         print("\nDirek etrafinda donus yapiliyor...", int(i), "derece\n------Cemberin merkezine gore konum-------"
               "\nKuzey:",iha.location.local_frame.north-x, "\nDogu:", iha.location.local_frame.east-y, 
               "\nCemberin merkezine uzaklik:{:.2f} metre".format(1000 * vincenty(anlik_konum_al(), direk_konumu)))
         if yon == 1: i = i + 360 / aralik;
         if yon == -1: i = i - 360 / aralik;
         time.sleep(zaman)   
    print("\nDonus tamamlandi","------Cemberin merkezine gore konum-------\nKuzey:",
               iha.location.local_frame.north - x, "\nDogu:", iha.location.local_frame.east - y)

def position(x, y, z, iha, aci):
    gercek_x = gercek_kuzey(x, y, aci)
    gercek_y = gercek_dogu(x, y, aci)
    msg = iha.message_factory.set_position_target_local_ned_encode(
      0,
      0, 0,
      mavutil.mavlink.MAV_FRAME_LOCAL_NED,
      0b0000111111111000,
      gercek_x, gercek_y, z,
      0, 0, 0,
      0, 0, 0,
      0, 0)
    iha.send_mavlink(msg)

## BEARING
bearing = calculate_initial_compass_bearing(direk1, direk2)
print("Pist Bearing: {:.2f} derece\n".format(bearing))

## YUKSELME
yukseklik = 1                  
armOlveYuksel(yukseklik)

## BASLANGICTAN DIREK2'YE GITME
iha_ilk_konum = (iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon)
iha_baslangic = LocationGlobalRelative(iha.location.global_frame.lat, iha.location.global_frame.lon, 0.5)
print("\nBaslangic konumu:", iha_ilk_konum)
iha_direk2_uzaklik = 1000 * vincenty(iha_ilk_konum, direk2)
iha_direk2_bearing = calculate_initial_compass_bearing(iha_ilk_konum, direk2)
fark_bearing_iha_direk2 = iha_direk2_bearing - bearing
direk2_konum_dogu = math.sin(math.radians(fark_bearing_iha_direk2)) * iha_direk2_uzaklik
direk2_konum_kuzey = math.cos(math.radians(fark_bearing_iha_direk2)) * iha_direk2_uzaklik
print("\nDirek 2 konum----------------\nKuzey:", direk2_konum_kuzey, "\nDogu:", direk2_konum_dogu)
print("\niha ile direk 2 arasi bearing:", iha_direk2_bearing, "\niha ile direk 2 arasi uzaklik:", iha_direk2_uzaklik,
      "\n\nDirek 2 baslangica gore konum------\nKuzey::", abs(direk2_konum_kuzey), "\nDogu:", abs(direk2_konum_dogu))
iha_direk2_yatay_uzaklik = 1000 * vincenty(anlik_konum_al(), direk2)*math.cos(math.radians(fark_bearing_iha_direk2))
iha_direk2_dusey_uzaklik = 1000 * vincenty(anlik_konum_al(), direk2)*math.sin(math.radians(fark_bearing_iha_direk2))
while abs(iha_direk2_yatay_uzaklik) > 0.2 and abs(iha_direk2_dusey_uzaklik) > 3.2:   
    position(direk2_konum_kuzey, direk2_konum_dogu + 3, -yukseklik, iha, bearing)
    print("\nDirek 2'ye ilerleniyor...\nDirek 2'ye yatay uzaklik: {:.2f} metre\nDirek 2'ye dusey uzaklik {:.2f} metre"
          .format(abs(iha_direk2_yatay_uzaklik), abs(iha_direk2_dusey_uzaklik)))
    time.sleep(1)
    iha_direk2_yatay_uzaklik = 1000*vincenty(anlik_konum_al(),direk2)*math.cos(math.radians(fark_bearing_iha_direk2))
    iha_direk2_dusey_uzaklik = 1000*vincenty(anlik_konum_al(),direk2)*math.sin(math.radians(fark_bearing_iha_direk2)) 
print("\niha istenilen noktaya vardi.")
print("Direk 2'ye yatay uzaklik: {:.2f} metre \nDirek 2'ye dusey uzaklik {:.2f} metre\n"
     .format(abs(iha_direk2_yatay_uzaklik), abs(iha_direk2_dusey_uzaklik)))

## DIREK 2'DE CEMBERSEL HAREKET      
cember(90, -90, -1, 3, direk2_konum_kuzey, direk2_konum_dogu, yukseklik, direk2)

## DİREK 2'DEN DİREK 3'E  GITME
iha_ikinci_konum = (iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon)
print("\nIkinci konum:",iha_ikinci_konum)
iha_direk3_uzaklik = 1000 * vincenty(iha_ikinci_konum, direk3)
iha_ilk_konum_direk3_bearing = calculate_initial_compass_bearing(iha_ilk_konum, direk3)
iha_direk3_bearing = calculate_initial_compass_bearing(iha_ikinci_konum, direk3)
fark_bearing_iha_ilk_konum_direk3 = iha_ilk_konum_direk3_bearing - bearing
fark_bearing_iha_direk3 = iha_direk3_bearing - bearing
direk3_konum_kuzey = 1000 * vincenty(iha_ilk_konum, direk3)*math.cos(math.radians(fark_bearing_iha_ilk_konum_direk3))
direk3_konum_dogu = 1000 * vincenty(iha_ilk_konum, direk3)*math.sin(math.radians(fark_bearing_iha_ilk_konum_direk3)) 
print("Direk 3 konum---------------\nKuzey:", direk3_konum_kuzey, "\nDogu:", direk3_konum_dogu)
print("\niha ile direk 3 arasi bearing:", iha_direk3_bearing, "\niha ile direk 3 arasi uzaklik:", iha_direk3_uzaklik,
      "\n\nDirek 3 baslangica gore konum-------\nKuzey:", abs(direk3_konum_kuzey), "\nDogu:", abs(direk3_konum_dogu))
iha_direk3_yatay_uzaklik = 1000 * vincenty(anlik_konum_al(), direk3)*math.cos(math.radians(fark_bearing_iha_direk3))
iha_direk3_dusey_uzaklik = 1000 * vincenty(anlik_konum_al(), direk3)*math.sin(math.radians(fark_bearing_iha_direk3))
while abs(iha_direk3_yatay_uzaklik) > 0.2 and abs(iha_direk3_dusey_uzaklik) > 3.2:
    position(direk3_konum_kuzey, direk3_konum_dogu + 3, -yukseklik, iha, bearing)
    print("\nDirek 3'e ilerleniyor...\nDirek 3'e yatay uzaklik: {:.2f} metre\nDirek 3'e dusey uzaklik {:.2f} metre"
          .format(abs(iha_direk3_yatay_uzaklik), abs(iha_direk3_dusey_uzaklik)))
    time.sleep(1)
    iha_direk3_yatay_uzaklik = 1000*vincenty(anlik_konum_al(),direk3)*math.cos(math.radians(fark_bearing_iha_direk3))
    iha_direk3_dusey_uzaklik = 1000*vincenty(anlik_konum_al(),direk3)*math.sin(math.radians(fark_bearing_iha_direk3))
print("iha istenilen noktaya vardi.")
print("Direk 3'e yatay uzaklik: {:.2f} metre \nDirek 3'e dusey uzaklik {:.2f} metre\n"
      .format(abs(iha_direk3_yatay_uzaklik), abs(iha_direk3_dusey_uzaklik)))

## DIREK 3'DE CEMBERSEL HAREKET      
cember(90, 450, 1, 3, direk3_konum_kuzey, direk3_konum_dogu, yukseklik, direk3)

## DİREK 3'DEN DİREK 1'E  GITME
iha_ucuncu_konum = (iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon)
print("\nUcuncu konum:",iha_ucuncu_konum)
iha_direk1_uzaklik = 1000 * vincenty(iha_ucuncu_konum, direk1)
iha_ilk_konum_direk1_bearing = calculate_initial_compass_bearing(iha_ilk_konum, direk1)
iha_direk1_bearing = calculate_initial_compass_bearing(iha_ucuncu_konum, direk1)
fark_bearing_iha_ilk_konum_direk1 = iha_ilk_konum_direk1_bearing - bearing
fark_bearing_iha_direk1 = iha_direk1_bearing - bearing
direk1_konum_kuzey = 1000 * vincenty(iha_ilk_konum, direk1)*math.cos(math.radians(fark_bearing_iha_ilk_konum_direk1))
direk1_konum_dogu = 1000 * vincenty(iha_ilk_konum, direk1)*math.sin(math.radians(fark_bearing_iha_ilk_konum_direk1)) 
print("Direk 1 konum----------------\nKuzey:", direk1_konum_kuzey, "\nDogu:", direk1_konum_dogu)
print("\niha ile direk 1 arasi bearing:", iha_direk1_bearing, "\niha ile direk 1 arasi uzaklik:", iha_direk1_uzaklik,
      "\n\nDirek 1 baslangica gore konum-------\nKuzey:", abs(direk1_konum_kuzey), "\nDogu:", abs(direk1_konum_dogu))
iha_direk1_yatay_uzaklik = 1000 * vincenty(anlik_konum_al(), direk1)*math.cos(math.radians(fark_bearing_iha_direk1))
iha_direk1_dusey_uzaklik = 1000 * vincenty(anlik_konum_al(), direk1)*math.sin(math.radians(fark_bearing_iha_direk1))
while abs(iha_direk1_yatay_uzaklik) > 0.2 and abs(iha_direk1_dusey_uzaklik) > 3.2:
    position(direk1_konum_kuzey, direk1_konum_dogu - 3, -yukseklik, iha, bearing)
    print("\nDirek 1'e ilerleniyor...\nDirek 1'e yatay uzaklik: {:.2f} metre\nDirek 1'e dusey uzaklik {:.2f} metre"
         .format(abs(iha_direk1_yatay_uzaklik), abs(iha_direk1_dusey_uzaklik)))
    time.sleep(1)
    iha_direk1_yatay_uzaklik = 1000*vincenty(anlik_konum_al(),direk1)*math.cos(math.radians(fark_bearing_iha_direk1))
    iha_direk1_dusey_uzaklik = 1000*vincenty(anlik_konum_al(),direk1)*math.sin(math.radians(fark_bearing_iha_direk1))
print("Istenilen noktaya varildi")
print("Direk 1'e yatay uzaklik: {:.2f} metre \nDirek 1'e dusey uzaklik {:.2f} metre\n"
      .format(abs(iha_direk1_yatay_uzaklik), abs(iha_direk1_dusey_uzaklik)))

## DIREK 1'DE CEMBERSEL HAREKET      
cember(270, 90, -1, 3, direk1_konum_kuzey, direk1_konum_dogu, yukseklik, direk1); time.sleep(1)

## BASLANGIC KONUMUNA DONUS
iha.simple_goto(iha_baslangic)
iha_konum = (iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon)
while 1000 * vincenty(iha_konum, iha_ilk_konum) > 0.5: 
    print("\nBaslangic konumuna donuluyor...\nkalan yol: {:.2f} metre".format(1000*vincenty(iha_konum, iha_ilk_konum)))
    iha_konum = (iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon)
    time.sleep(1)
iha.mode = "LAND"
while not iha.armed == False: print("\nInis yapılıyor"); time.sleep(0.5);
