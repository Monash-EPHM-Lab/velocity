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


for _ in range(6000,6500,5):
    pt = points[_]

    psd = rpsd('5k6_wtr_wait.csv')

    if pt.get_fft()[0] == None:
           #print(_)
           continue

    hach_vem = pt.get_hach_vel()   
   
    vels = [SdPoint.binconv * i for i in range(128)]

    plfft = pt.get_fft()
    psdfft = [x for x in plfft]
    psdfft = [binn/(weight) for binn,weight in zip(plfft, psd)]
    psdfft = gf(psdfft, 2)
    psdfft = [x for x in psdfft]



    alvem, alves, maxvel,dfft = pt.cannym(psd)
    
    dfft = [x for x in dfft]

    ax.plot(vels, plfft, label = 'raw fft')
    
    coax.plot(vels, dfft, label = 'dfft', c = 'g')
    ax.plot(vels, psdfft, label = 'whitened fft')

    ax.vlines(hach_vem, 0 , 1E12, label = 'hach vel')
    
    ax.vlines(alvem, 0 , 1E12, colors = "r", label = 'bosl fft')
    coax.hlines(0,0,1E12, colors = 'k')
    
    coax.plot([0,64*SdPoint.binconv],[0,-10000])
    #coax.hlines(pt.ctlow,0,1E12, colors = 'k')
   
    ax.add_patch(patches.Rectangle((0,0), 4000 , pt.thrsh, facecolor="#ed092040"))


    ax.set_xlim(0,64*SdPoint.binconv)
    ax.set_ylim(10,25000000)
    coax.set_xlim(0,64*SdPoint.binconv)
    coax.set_ylim(-25000,25000)

    ax.set_title(str(pt.get_time()) +"  |   " + str(pt.get_id()))
    ax.set_xlabel("Velocity (mm/s)")
    ax.set_yscale('symlog')
    ax.set_ylabel("Amplitude")
    ax.legend(loc = 'upper right')
    plt.savefig('out\\' + str(_) + '.png', dpi = 300)

    
    ax.clear()
    coax.clear()

# #for i in range(0,100,1):


plt.close(fig)
figu, axv  = plt.subplots()
coaxvu = axv.twinx()

points = [pt for pt in points if pt.cannym(rpsd('5k6_air.csv'))[0] != 0]
# points = [pt for pt in points if pt.algoM()[0] > 300]



tv = [x.get_time() for x in points]
hv = [x.get_hach_vel() for x in points]
dv = [1000*x.get_hach_depth() for x in points]
bv = [x.get_vem() for x in points]    
bsv = [x.get_ves() for x in points]  
bav = [x.get_vea() for x in points]      
tvd = [x.cannym(rpsd('5k6_air.csv')) for x in points]
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
axv.set_ylim([0,2000])

axv.scatter(tv,av, c=powr, cmap="inferno", s = 1)

coaxv = ax.twinx()
axv.scatter(tv,hv, c = '#5802e340', s = 1)
#ax2.scatter(tv,dv, c = '#5802e340', s = 1)
coaxv.set_ylim([0,3000])
dstart = datetime(2020,9,15)
dend = datetime(2020,9,29)
axv.set_xlim(dstart,dend)
fig.autofmt_xdate()
#ax.set_xlim([0,2000])



axv.set_ylabel('Velocity (mm/s)')
coaxv.set_ylabel('Depth (mm)')
axv.set_xlabel('Date')


#plt.savefig('time_depth.png', dpi = 600)
# plt.cla()
                   


plt.show()

     
#plt.scatter([point.get_hach_vel() for point in points],[point.get_vem() for point in points], s = 4)

