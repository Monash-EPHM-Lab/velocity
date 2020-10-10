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

load_data(points)
#load_test(points)


fig, ax = plt.subplots()


class pointstruct:
    def __init__(self, a, u):
        self.v = a[0]
        self.u = u
        self.p = a[2]
    # def __str__(self):
        # return str(self.u)
    # def __repr__(self):
        # return str(self.u)

bin_size = 200

ptstruct = [pointstruct(x.algoM(),x.get_hach_vel()) for x in points]
# ptstruct = [pointstruct([x.get_vem(),None,x.get_vea()],x.get_hach_vel()) for x in points]

velpartition  = [list() for i in range(5000//bin_size)]

for pt in ptstruct:
    binn = int(pt.u//bin_size)
    velpartition[binn].append(pt)

# for i in velpartition:
    # print(len(i))
    
def RMS(pts):
    DUp = 0.10 

    ampmean  = np.mean([x.p for x in pts])
    
    pts = [x for x in pts if (x.v > 0 and x.u > 0 )] #and 3*x.p > ampmean
    
    N = len(pts)
    if N == 0:
        return -1, -1

    sumation = 0
    for pt in pts:
        sumation += (pt.v/pt.u - 1)**2
        
    R = math.sqrt(sumation/N)
    
    errsum = 0
    for pt in pts:
        DU = DUp*pt.u
        errsum += ((pt.v/pt.u - 1)*((pt.v/pt.u)*(DU/pt.u)))**2
    
    DR = 1/(R *N)*math.sqrt(errsum)
        
    return round(R, 2), round(DR, 2), N 



for i,rnge in enumerate(velpartition):
    # amps = [x.p for x in rnge]
    # vels = [x.u for x in rnge]
    # mn = sum(amps)/len(amps)
    # plt.scatter(vels,amps)
    # plt.hlines([mn*i/2 for i in range(10)], 0, 2000)
    # plt.show()
    print(str(RMS(rnge)) + "    " +str(i*bin_size))
    

# #sv = [np.mean(sv) if x > np.mean(sv) else x for x in sv]
# powr = [np.mean(powr) if x > np.mean(powr) else x for x in powr]   


# batches
# # for a,b in zip(av,bv):
    # # print(a)
    # # print(b)
    # # input()

# # ax.scatter(powr,bav, c = '#348feb', s = 8)




# # ax.scatter(tv,av, c=powr, cmap="Blues", s = 8)

# # ax.scatter(tv,hv, c = '#f0952640', s = 8)
# #ax.scatter(tv,dv, c = '#5802e340', s = 8)
# # dstart = datetime(2020,7,27)
# # dend = datetime(2020,9,27)
# # ax.set_xlim(dstart,dend)

# # ax.plot([0,2000],[0,2000], c = 'r' )
# # ax.scatter(hv,av, c=powr, cmap="Blues", s = 8)



# # plt.savefig('out\\' + str(i) + '.png', dpi = 300)
# # plt.cla()
                   


# plt.show()

     
#plt.scatter([point.get_hach_vel() for point in points],[point.get_vem() for point in points], s = 4)

