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
#ax2 = ax.twinx()
##






# ave = 5
# apoints = []
# for i in range(len(points)):
        # try:
                # pt = SdPoint()
                # pt.collect(points[i:i+ave])
                # apoints.append(pt)
        # except IndexError:
                # pass
        




# for _ in range(10000,14000,10):
        # pt = points[_]

        # if pt.get_fft()[0] == None:
               # continue

        # hach_vem = pt.get_hach_vel()
        # vem = pt.get_vemc()
        # ves = pt.get_vesc()
       
       
        # vels = [SdPoint.binconv * i for i in range(128)]

        # plfft = pt.get_fft()
        # # # plfft[0] = 0
        # # #plfft[1] = 0
        # pldfft = [val/weight for (val,weight) in zip(plfft, psd_wtr)]
        # # pldfft = gf(pldfft, sigma = 1)
        # pldfft = gf(pldfft, sigma = 1)
        # pldfft = [x for x in pldfft]
        
        # #dfft = pt.canny()
       
        # #plafft = [val/weight for (val,weight) in zip(plfft, psd_wtr)]

        # alvem, alves, dfft = pt.algoM()
 

        # #ax.plot(vels, plfft)
        # ax.plot(vels, pldfft)
        # #ax.plot(vels, dfft)
        # #ax.plot(vels, plafft)
        # ax.vlines(hach_vem, 0 , 1E12)
        # #ax.vlines(vem, 0 , 1E12, colors = "g")
        # ax.vlines(alvem, 0 , 1E12, colors = "r")
       
        # #ax.add_patch(patches.Rectangle((0,0), 5*SdPoint.binconv , 1E12, facecolor="#ed092040"))
        # ax.add_patch(patches.Rectangle((0,0), 4000 , 8000, facecolor="#ed092040"))
        # #ax.add_patch(patches.Rectangle((vem - ves,0), 2*ves , 1E12, facecolor="#e8a40540"))
        # ax.add_patch(patches.Rectangle((alvem - alves,0), 2*alves , 1E12, facecolor="#34eb7440"))
        # ax.set_xlim(0,32*SdPoint.binconv)
        # ax.set_ylim(0,250000)

        # ax.set_title(pt.get_time())
        # ax.set_xlabel("Velocity (mm/s)")
        # ax.set_ylabel("Amplitude")
        # # plt.show()
        # # exit()
        # plt.savefig('out\\' + str(pt.get_id()) + '.png', dpi = 300)

        # plt.cla()

#for i in range(0,100,1):

#points = [pt for pt in points if pt.algoM()[0] != 0]
# points = [pt for pt in points if pt.algoM()[0] > 300]



tv = [x.get_time() for x in points]
hv = [x.get_hach_vel() for x in points]
dv = [1000*x.get_hach_depth() for x in points]
bv = [x.get_vem() for x in points]    
bsv = [x.get_ves() for x in points]  
bav = [x.get_vea() for x in points]      
tvd = [x.algoM() for x in points]
av = [x[0] for x in tvd]
sv = [x[1] for x in tvd]
powr = [x[2] for x in tvd]




def intime(x,ds,de):
    
    if x > ds:
        if x < de:
            return 1
    return 0
    

# ct = [x for x in tv if intime(x,datetime(2020,9,18),datetime(2020,9,19))]
# print(len(ct))


def slog(x):
    try:
        r = math.log(x)
    except ValueError:
        r = 0
    return r

# #sv = [np.mean(sv) if x > np.mean(sv) else x for x in sv]
 
powr = [-slog(x) for x in powr]   


# batches
# # for a,b in zip(av,bv):
    # # print(a)
    # # print(b)
    # # input()

# # ax.scatter(powr,bav, c = '#348feb', s = 8)


###################################this clips some data
ax.set_ylim([0,1500])

ax.scatter(tv,bv, c=powr, cmap="inferno", s = 1)

ax2 = ax.twinx()
ax.scatter(tv,hv, c = '#5802e340', s = 1)
#ax2.scatter(tv,dv, c = '#5802e340', s = 1)
ax2.set_ylim([0,3000])
dstart = datetime(2020,9,22)
dend = datetime(2020,10,15)
ax.set_xlim(dstart,dend)
fig.autofmt_xdate()
#ax.set_xlim([0,2000])



ax.set_ylabel('Velocity (mm/s)')
ax2.set_ylabel('Depth (mm)')
ax.set_xlabel('Date')


#plt.savefig('time_depth.png', dpi = 600)
# plt.cla()
                   


plt.show()

     
#plt.scatter([point.get_hach_vel() for point in points],[point.get_vem() for point in points], s = 4)

