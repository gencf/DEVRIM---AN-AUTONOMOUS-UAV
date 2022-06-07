# x ve y'deki hatalar ayri olarak degerlendirilmistir !!!


import math
import time


class PID:

    def __init__(self, coz_x=600, coz_y=450):
        # DEFINE CONSTANTS OF PID ELEMENT

        self.kphx = 0.001 # kp of x axis
        self.kihx = 0.0005 # ki of x axis

        self.kdhx = 0.001 # kd of x axis (start: 0)

        self.kphy = 0.001 # kp of y axis
        self.kihy = 0.0005 # ki of y axis

        self.kdhy = 0.001 # kd of y axis (start: 0)

        # DEFINE RESOLUTION
        self.cozunurluk_x = coz_x
        self.cozunurluk_y = coz_y
    
        self.ixerror = 0
        self.iyerror = 0
        self.pxerrorh = 0    
        self.pyerrorh = 0   

    def update(self, ort_x, ort_y):
        # CALCULATE EUCLIDEAN DISTANCE ACCORDING TO ERROR IN X AND Y AXIS
        errorx = ort_x - (self.cozunurluk_x / 2.0)  
        errory = ort_y - (self.cozunurluk_y / 2.0)

        # PREVENT INTEGRAL ERROR TO INCREASE SO MUCH
	k = 0.09
        if (errorx + self.ixerror) * self.kihx < k and (errorx + self.ixerror) * self.kihx > -k:
            self.ixerror = errorx + self.ixerror  

        if (errory + self.iyerror) * self.kihy < k and (errory + self.iyerror) * self.kihy > -k:
            self.iyerror = errory + self.iyerror

        errorxdh = errorx - self.pxerrorh
        errorydh = errory - self.pyerrorh

	print("errorx = {}\nerrorxd = {}\nixerroryd = {}".format(self.kphx * errorx, self.kdhx * errorxdh, self.kihx * self.ixerror))
	print("errory = {}\nerroryd = {}\niyerroryd = {}".format(self.kphy * errory, self.kdhy * errorydh, self.kihy * self.iyerror))

        pidx = self.kphx * errorx + self.kihx * self.ixerror + self.kdhx * errorxdh
        pidy = self.kphy * errory + self.kihy * self.iyerror + self.kdhy * errorydh

        # Onceki error degerini guncelle
        self.pxerrorh = errorx
        self.pyerrorh = errory

        if pidx > 1:
            pidx = 1

        elif pidx < -1:
            pidx = -1

        if pidy > 1:
            pidy = 1

        elif pidy < -1:
            pidy = -1
	
	print("pidx = {}\npidy = {}\n".format(pidx, pidy))

        vx = -pidy  # y yonundeki hizi
        vy = pidx  # x yonundeki hizi
	time.sleep(0.5)
        return vx, vy, errorx, errory


