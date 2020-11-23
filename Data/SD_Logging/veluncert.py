from datetime import datetime,timedelta
import numpy as np
import matplotlib.dates as mdates
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter as gf
import random
from velutil import *

points = []


#print(psd_wtr)

load_data(points, '000')
#load_test(points)


##000 hach const filter
def no_data_rem(pt):
    pt_time = pt.get_time()
    
    del_times = [[datetime(2020,7,29, hour = 12),datetime(2020,7,29, hour = 19)],
                 [datetime(2020,9,8, hour = 10),datetime(2020,9,8, hour = 20)],
                 [datetime(2020,9,22, hour = 8),datetime(2020,9,22, hour = 17)],
                 [datetime(2020,9,29, hour = 9),datetime(2020,9,29, hour = 16)],
                 [datetime(2020,10,6, hour = 8),datetime(2020,10,7, hour = 15)],
                ]
    for rang in del_times:
        if  rang[0] < pt_time < rang[1]:
            return False
    return True
    
    
points = [pt for pt in points if no_data_rem(pt)]    
points = [pt for pt in points if (pt.cannym(rpsd('5K6_wtr_wait.csv'))[0] != 0)]
points = [pt for pt in points if not(pt.cannym(rpsd('5K6_wtr_wait.csv'))[0] > 250 and pt.cannym(rpsd('5K6_wtr_wait.csv'))[2] < 3)]

class pointstruct:
    def __init__(self, a, u):
        self.v = a[0]
        self.u = u
        self.p = a[2]
    # def __str__(self):
        # return str(self.u)
    # def __repr__(self):
        # return str(self.u)

bin_size = 250

ptstruct = [pointstruct(x.cannym(rpsd('5K6_wtr_wait.csv')),x.get_hach_vel()) for x in points]


velpartition  = [list() for i in range(5000//bin_size)]

for pt in ptstruct:
    binn = int(pt.u//bin_size)
    velpartition[binn].append(pt)

# for i in velpartition:
    # print(len(i))
    
def RMS(pts):
    DUp = 0.10 

    maxx = max([x.u for x in pts])
    minn = min([x.u for x in pts])
    midd = 0.5*(maxx + minn)

    ampmean  = np.mean([x.p for x in pts])
    
    pts = [x for x in pts if (x.v > 0 and x.u > 0 )] #and 3*x.p > ampmean
    
    N = len(pts)
    if N == 0:
        return -1, -1

    sumation = 0
    for pt in pts:
        sumation += (pt.v/pt.u - 1)**2
        
    R = math.sqrt(sumation/N)
    
    
    meanerr =0
    for pt in pts:
        meanerr += (pt.v - pt.u)
    M = meanerr/(N*midd)
    
    
    
    # errsum = 0
    # for pt in pts:
        # DU = DUp*pt.u
        # errsum += ((pt.v/pt.u - 1)*((pt.v/pt.u)*(DU/pt.u)))**2
    
    # DR = 1/(R *N)*math.sqrt(errsum)
        
    return round(R, 2), round(M, 2), N 



for i,rnge in enumerate(velpartition):
    # amps = [x.p for x in rnge]
    # vels = [x.u for x in rnge]
    # mn = sum(amps)/len(amps)
    # plt.scatter(vels,amps)
    # plt.hlines([mn*i/2 for i in range(10)], 0, 2000)
    # plt.show()
    print(str(RMS(rnge)) + "    " +str(i*bin_size))
    


