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

with open('HACH_data_2.csv', newline='') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
	for i,row in enumerate(spamreader):
		if i < 30:
			continue
		date_string = row[0] + " " + row[2]
		hack_date.append(datetime.strptime(date_string, '%d/%b/%y %H:%M'))
		hack_depth.append(float(row[12])*1000)
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



PVels3Adj = []
PVels4Adj = []
for val in PVels[3]:
	PVels3Adj.append(1750-val)
for val in PVels[4]:
	PVels4Adj.append(val-2200)

ave = []
for i,val in enumerate(PVels[0]):
	ave.append((PVels[0][i] + PVels[1][i] + PVels[2][i])/3)


# plt.scatter(Amps[0],Vels[0])
# plt.scatter(Amps[1],Vels[1])
# plt.scatter(Amps[2],Vels[2])
# plt.scatter(Amps[3],Vels[3])
# plt.scatter(Amps[4],Vels[4])


fig, ax1 = plt.subplots()

ax2 = ax1.twinx()

ax1.scatter(times,PVels[3])
ax1.scatter(hack_date, hack_vel)
#ax2.scatter(hack_date, hack_depth, c = "g")
#plt.scatter(times,PVels[2])
#plt.scatter(times,PVels[1])
#plt.scatter(times,PVels[0])
ax1.set_xlabel("Date")
ax1.set_ylabel("Velocity (mm/s)")
ax2.set_ylim(0,2000)
ax2.set_ylabel("Depth (mm)")


dstart = datetime(2020,6,20)
dend = datetime(2020,7,10)
plt.xlim(dstart, dend) 



plt.format_xdata = mdates.DateFormatter('%Y-%m-%d')
#plt.autofmt_xdate()

plt.show()
