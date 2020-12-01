import serial
import math
import numpy as np
import random
import time
from velutil import *
import matplotlib.pyplot as plt


psdfp = '5k6_wtr_wait.csv'

points = []
load_data(points, '000',10,11)

ATvems = []
PYvems = []

ATamps = []
PYamps = []


    
for _ in range(200):
    print(_)
    pt = random.choice(points)



    mean_indx, std_indx, amp ,rdfft, dfft, ret_fftpsd, ctlow = pt.cannym(rpsd(psdfp))

    PYvems.append(mean_indx)
    PYamps.append(amp)


    ser = serial.Serial('COM25', timeout = 1)  # open serial port
    print(ser.name)         # check which port was really used

    x = ''
    while( x != b'R'):
        x = ser.read()

    for i in range(128):
        ser.write(str(pt.fft[i]).encode('utf-8'));
        ser.write(str(',').encode('utf-8'));

    s = ser.readline()
    ATfft = s.split(b',')
    ATfft = [float(x) for x in ATfft[0:-1]]


    s = ser.readline()
    print(s)
    try:
        ATvems.append(float(s))
    except ValueError:
        ATvems.append(0)
    s = ser.readline()
    print(s)
    try:
        ATamps.append(float(s))
    except ValueError:
        ATamps.append(0)


    print(mean_indx)
    print(amp)
    ser.close()

plt.plot([0,1500],[0,1500])
plt.scatter(PYvems, ATvems)
plt.show()










