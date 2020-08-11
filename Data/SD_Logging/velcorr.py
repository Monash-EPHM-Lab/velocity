#csv file notes
#first data in creek starts at row 46
#the code where one sample was taken per average has logs from line 5421 onwards


import csv
from datetime import datetime,timedelta
import numpy as np
import matplotlib.dates as mdates
import matplotlib.pyplot as plt

times = []
bat = []


VeM = []
VeS = []

hack_date = []
hack_depth = []
hack_vel = []

with open('hach.csv', newline='') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
	for i,row in enumerate(spamreader):
		if i < 30:
			continue
		date_string = row[0] + " " + row[2]
		hack_date.append(datetime.strptime(date_string, '%d/%b/%y %H:%M'))
		hack_depth.append(float(row[12]))
		hack_vel.append(float(row[17])*1000)

with open('log.csv', newline='') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
	for i,row in enumerate(spamreader):
		if i < 88:
			continue

		times.append(datetime.strptime(row[0], '%d/%m/%y %I:%M:%S %p')+timedelta(hours=10))
		##bat.append(float(row[1]))
		VeM.append(float(row[7])* 1.15) #1.15 factor is due to angle of transducers.
		VeS.append(float(row[8])* 1.15) #1.15 factor is due to angle of transducers.



corr_hach = [];
corr_bosl = [];

hi = 0;

boslT = times[24]
hachT  = hack_date[0]
Dtime = boslT - hachT

for i,val in enumerate(VeM):
    if(i < 24):
        continue
    boslT = times[i]
    hachT  = hack_date[hi]
    Dtime = boslT - hachT

    while (Dtime >= timedelta(minutes=1)):
        boslT = times[i]
        hachT  = hack_date[hi]
        Dtime = boslT - hachT
        hi += 1
    hi += 1
    if (hi >= len(hack_date)):
        break
    #print(times[i], " ", hack_date[hi-1])
    corr_hach.append(hack_vel[hi-1])
    corr_bosl.append(VeM[i])



plt.scatter(corr_hach,corr_bosl, s = 4)
plt.plot([0,2000],[0,2000], c = 'r' )


plt.xlabel("Hack Velocity (mm/s)")
plt.ylabel("BoSL Velocity (mm/s)")

plt.xlim(0,1200)
plt.ylim(0,4000)


plt.format_xdata = mdates.DateFormatter('%Y-%m-%d')
#plt.autofmt_xdate()

plt.show()
