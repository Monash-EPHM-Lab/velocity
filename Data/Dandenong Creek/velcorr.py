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


Vel0 = []
Vel1 = []
Vel2 = []
Vel3 = []
Vel4 = []

Vels = [Vel0, Vel1, Vel2, Vel3, Vel4]

PVel0 = []
PVel1 = []
PVel2 = []
PVel3 = []
PVel4 = []

PVels = [PVel0, PVel1, PVel2, PVel3, PVel4]

Amp0 = []
Amp1 = []
Amp2 = []
Amp3 = []
Amp4 = []

Amps = [Amp0, Amp1, Amp2, Amp3, Amp4]

hack_date = []
hack_depth = []
hack_vel = []

with open('HACH_data.csv', newline='') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
	for i,row in enumerate(spamreader):
		if i < 30:
			continue
		date_string = row[0] + " " + row[2]
		hack_date.append(datetime.strptime(date_string, '%d/%b/%y %H:%M'))
		hack_depth.append(float(row[12]))
		hack_vel.append(float(row[17])*1000)

with open('VELOCITY_SITE_TES.csv', newline='') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
	for i,row in enumerate(spamreader):
		if i < 88:
			continue

		times.append(datetime.strptime(row[0], '%d/%m/%y %I:%M:%S %p')+timedelta(hours=10))
		##bat.append(float(row[1]))
		Vel0.append(float(row[6]))
		Amp0.append(float(row[7]))
		Vel1.append(float(row[8]))
		Amp1.append(float(row[9]))
		Vel2.append(float(row[10]))
		Amp2.append(float(row[11]))
		Vel3.append(float(row[12]))
		Amp3.append(float(row[13]))

for set,lst in enumerate(Vels):		
	for i,val in enumerate(Vels[set]):
		if(Amps[set][i] > 20):
			PVels[set].append(1.15*val) #1.15 factor is due to angle of transducers.
		else:
			PVels[set].append(float("nan"))

corr_hach = [];
corr_bosl = [];

hi = 0;

boslT = times[24]
hachT  = hack_date[0]
Dtime = boslT - hachT

for i,val in enumerate(PVels[3]):
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
    corr_bosl.append(PVels[3][i])



plt.scatter(corr_hach,corr_bosl)
plt.plot([0,2000],[0,2000], c = 'r')
#plt.scatter(hack_date, hack_vel)
#plt.scatter(times,PVels[3])

plt.xlabel("Hack Velocity (mm/s)")
plt.ylabel("BoSL Velocity (mm/s)")

##
##dstart = datetime(2020,6,20)
##dend = datetime(2020,6,25)
##plt.xlim(dstart, dend) 
plt.xlim(0,1200)
plt.ylim(0,4000)


plt.format_xdata = mdates.DateFormatter('%Y-%m-%d')
#plt.autofmt_xdate()

plt.show()
