
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
		hack_depth.append(float(row[12])*1000)
		hack_vel.append(float(row[17])*1000)

with open('log.csv', newline='') as csvfile:
	spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
	for i,row in enumerate(spamreader):
		

		times.append(datetime.strptime(row[0], '%d/%m/%y %I:%M:%S %p')+timedelta(hours=10))
		##bat.append(float(row[1]))
		
		VeM.append(float(row[7])*1.15) #1.15 is cosine losses of 30 deg
		VeS.append(float(row[8])*1.15) #1.15 is cosine losses of 30 deg





fig, ax1 = plt.subplots()

ax2 = ax1.twinx()

ax1.scatter(times,VeM , s = 2)
ax1.scatter(hack_date, hack_vel, c = '#f9730610', s = 2)
#ax2.scatter(hack_date, hack_depth, c = "g")

ax1.set_xlabel("Date")
ax1.set_ylabel("Velocity (mm/s)")
ax2.set_ylim(0,2000)
ax2.set_ylabel("Depth (mm)")


dstart = datetime(2020,7,8)
dend = datetime(2020,7,16)
plt.xlim(dstart, dend) 



plt.format_xdata = mdates.DateFormatter('%Y-%m-%d')
#plt.autofmt_xdate()

plt.show()
