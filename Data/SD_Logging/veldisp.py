from datetime import datetime,timedelta
import numpy as np
import matplotlib.dates as mdates
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.ndimage import gaussian_filter as gf
import random
from velutil import *

points = []


load_data(points, '000')
#load_test(points)
fig  = plt.figure()
gs = gridspec.GridSpec(2, 1, figure=fig)
ax = fig.add_subplot(gs[0, :])

coax = fig.add_subplot(gs[1, :])


# for _ in range(6800,7300,4):
    # pt = points[_]

    # psd = rpsd('5k6_wtr_wait.csv')

    # if pt.get_fft()[0] == None:
           # #print(_)
           # continue

    # hach_vem = pt.get_hach_vel()   
   
    # vels = [SdPoint.binconv * i for i in range(128)]

    # plfft = pt.get_fft()
    # psdfft = [x for x in plfft]
    # psdfft = [binn/(weight) for binn,weight in zip(plfft, psd)]
    # psdfft = gf(psdfft, 3)
    # psdfft = [x for x in psdfft]



    # alvem, alves, alvea = pt.algoM(psd)


    # ax.plot(vels, plfft, label = 'raw fft')
    

    # ax.plot(vels, psdfft, label = 'whitened fft')

    # ax.vlines(hach_vem, 0 , 1E12, label = 'hach vel')
    
    # ax.vlines(alvem, 0 , 1E12, colors = "r", label = 'bosl fft')
    # coax.hlines(0,0,1E12, colors = 'k')

    # #coax.hlines(pt.ctlow,0,1E12, colors = 'k')
   
    # ax.add_patch(patches.Rectangle((0,0), 4000 , pt.thrsh, facecolor="#ed092040"))


    # ax.set_xlim(0,64*SdPoint.binconv)
    # ax.set_ylim(10,25000000)
    # coax.set_xlim(0,64*SdPoint.binconv)
    # coax.set_ylim(-25000,25000)

    # ax.set_title(str(pt.get_time()) +"  |   " + str(pt.get_id()))
    # ax.set_xlabel("Velocity (mm/s)")
    # ax.set_yscale('symlog')
    # ax.set_ylabel("Amplitude")
    # ax.legend(loc = 'upper right')
    # plt.savefig('out\\' + str(_) + '.png', dpi = 300)

    
    # ax.clear()
    # coax.clear()

# #for i in range(0,100,1):


plt.close(fig)

fig  = plt.figure()
gs = gridspec.GridSpec(2, 1, figure=fig)
ax = fig.add_subplot(gs[0, :])

coax = fig.add_subplot(gs[1, :])


points = [pt for pt in points if pt.algoM(rpsd('5K6_wtr_wait.csv'))[0] != 0]
# points = [pt for pt in points if pt.algoM()[0] > 300]



tv = [x.get_time() for x in points]
hv = [x.get_hach_vel() for x in points]
dv = [1000*x.get_hach_depth() for x in points]
bv = [x.get_vem() for x in points]    
bsv = [x.get_ves() for x in points]  
bav = [x.get_vea() for x in points]      
tvd = [x.algoM(rpsd('5K6_wtr_wait.csv')) for x in points]
av = [x[0] for x in tvd]
sv = [x[1] for x in tvd]
powr = [x[2] for x in tvd]


mpowr = np.mean(powr)
powr = [x if x < mpowr else mpowr for x in powr]

###################################this clips some data
ax.set_ylim([0,2000])
coax.set_ylim([0,2000])

ax.scatter(tv,av, c = powr, s = 4)


coax.scatter(tv,hv, c = '#5802e340', s = 2)

#coaxv.scatter(tv,dv, c = '#5802e340', s = 1)

dstart = datetime(2020,9,15)
dend = datetime(2020,9,29)
ax.set_xlim(dstart,dend)
coax.set_xlim(dstart,dend)
fig.autofmt_xdate()
#ax.set_xlim([0,2000])



ax.set_ylabel('Velocity BoSL (mm/s)')
coax.set_ylabel('Velocity HACH (mm/s)')
ax.set_xlabel('Date')
coax.set_xlabel('Date')

plt.show()

plt.close(fig)

# figu, axv = plt.subplots()

# axv.set_ylim([0,2000])
# axv.set_xlim([0,2000])
# axv.set_ylabel('BoSL Velocity (mm/s)')
# axv.set_xlabel('HACH Velocity (mm/s)')
# axv.scatter(hv,av, c=powr, cmap="inferno", s = 1)
# axv.plot([0,2000],[0,2000], c = 'r')


#plt.savefig('time_depth.png', dpi = 600)
# plt.cla()
                   




















# for _ in range(6800,7300,4):
    # pt = points[_]

    # psd = rpsd('5k6_wtr_wait.csv')

    # if pt.get_fft()[0] == None:
           # #print(_)
           # continue

    # hach_vem = pt.get_hach_vel()   
   
    # vels = [SdPoint.binconv * i for i in range(128)]

    # plfft = pt.get_fft()
    # psdfft = [x for x in plfft]
    # psdfft = [binn/(weight) for binn,weight in zip(plfft, psd)]
    # psdfft = gf(psdfft, 3)
    # psdfft = [x for x in psdfft]



    # alvem, alves, maxvel,dfft, ffft = pt.cannym(psd)
    
    # dfft = [x for x in dfft]
    # ffft = [x for x in ffft]

    # ax.plot(vels, plfft, label = 'raw fft')
    
    # coax.plot(vels, dfft, label = 'dfft', c = 'g')
    # coax.plot(vels, ffft, label = 'ffft', c = 'r')
    # ax.plot(vels, psdfft, label = 'whitened fft')

    # ax.vlines(hach_vem, 0 , 1E12, label = 'hach vel')
    
    # ax.vlines(alvem, 0 , 1E12, colors = "r", label = 'bosl fft')
    # coax.hlines(0,0,1E12, colors = 'k')
    
    # thresholds = [min(pt.ctlow/p,pt.ctlow) for p in psd]
    
    # coax.plot(vels,thresholds)
    # #coax.hlines(pt.ctlow,0,1E12, colors = 'k')
   
    # ax.add_patch(patches.Rectangle((0,0), 4000 , pt.thrsh, facecolor="#ed092040"))


    # ax.set_xlim(0,64*SdPoint.binconv)
    # ax.set_ylim(10,25000000)
    # coax.set_xlim(0,64*SdPoint.binconv)
    # coax.set_ylim(-25000,25000)

    # ax.set_title(str(pt.get_time()) +"  |   " + str(pt.get_id()))
    # ax.set_xlabel("Velocity (mm/s)")
    # ax.set_yscale('symlog')
    # ax.set_ylabel("Amplitude")
    # ax.legend(loc = 'upper right')
    # plt.savefig('out\\' + str(_) + '.png', dpi = 300)

    
    # ax.clear()
    # coax.clear()



    