# Single Color RGB565 Blob Tracking Example
#
# This example shows off single color RGB565 tracking using the OpenMV Cam.

import sensor, image, time
from pyb import LED,SPI

red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
ir_led    = LED(4)

for i in range(0,1):
    blue_led.on()
    time.sleep(150)
    blue_led.off()
    time.sleep(100)
    blue_led.on()
    time.sleep(150)
    blue_led.off()
    time.sleep(600)

threshold_index = 4
blue_index=5

thresholds = [(0, 100, 4, 127, 4, 127), # red_thresholds
              (30, 100, -64, -8, -32, 32), # green_thresholds
              (0, 30, 0, 64, -128, 0),# blue_thresholds
              (30,100,-30,30,-30,30),
              (0, 49, -83, 38, -6, 58),#black_thresholds
              (0, 100, -55, 35, -128, -4)]#blue

(0, 100, 3, 127, -1, 127)#。。。
rois=[(0,0,160,120)]
ROIS=[(20,20,120,80)]
f_x = (2.8 / 3.984) * 160
f_y = (2.8 / 2.952) * 120
c_x = 160 * 0.5
c_y = 120 * 0.5
height=0
width=0

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
#sensor.skip_frames(30)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False,value=(150,150,150)) # must be turned off for color tracking
sensor.set_auto_exposure(False, value = 1400)
clock = time.clock()

class Point(object):
    def get_axis(self,x,y):
        self.x=x
        self.y=y

rgb=Point()
gray=Point()

while(True):
    clock.tick()
    pointlist=[0XFE]
    a=0
    rgbbloblist=[]
    graybloblist=[]
    img = sensor.snapshot()
    for r in rois:
        blobsrgb=img.find_blobs([thresholds[blue_index]],roi=r, pixels_threshold=20,  merge=True)
        if blobsrgb:
            largest_blobrgb=max(blobsrgb,key=lambda b:b.pixels())
            rgbbloblist.append(largest_blobrgb)
            rgb.get_axis(largest_blobrgb.cx(),largest_blobrgb.cy())
        else:
            rgb.get_axis(253,253)
    for rs in ROIS:
        blobsgray=img.find_blobs([thresholds[threshold_index]], roi=rs, pixels_threshold=20, merge=True)
        if blobsgray:
            largest_blobgray=max(blobsgray,key=lambda b:b.pixels())
            graybloblist.append(largest_blobgray)
    #print(clock.fps())
            gray.get_axis(largest_blobgray.cx(),largest_blobgray.cy())
        else:
            gray.get_axis(253,253)
    if len(rgbbloblist)!=0:
        print("rgbwidth",largest_blobrgb.w())
        print("rgbheight",largest_blobrgb.h())
        img.draw_rectangle(largest_blobrgb.rect(),color=(255,0,0))
        img.draw_cross(largest_blobrgb.cx(), largest_blobrgb.cy(),color=(255,0,0))
    if len(graybloblist)!=0:
        print("graywidth",largest_blobgray.w())
        print("grayheight",largest_blobgray.h())
        img.draw_rectangle(largest_blobgray.rect(),color=(0,0,255))
        img.draw_cross(largest_blobgray.cx(), largest_blobgray.cy(),color=(0,0,255))

    print(rgb.x,rgb.y,gray.x,gray.y)
    img.draw_string(0,0,"FPS=%s"%clock.fps())

    pointlist.append(gray.x)
    pointlist.append(gray.y)
    pointlist.append(rgb.x)
    pointlist.append(rgb.y)
    for i in range(0,4):
        a+=pointlist[i+1]%10
    pointlist.append(a)
    pointlist.append(0XFF)
    time.sleep(5)
    #print(pointlist)

