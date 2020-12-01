import csv
import math

fp = '5K6_wtr_wait.csv'

psd = []

with open(fp, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for i,row in enumerate(reader):
        if (i == 0):
            continue
        if (i > 9):
            break
        psd = psd + row[0:16]


psd = [math.log(float(x)) for x in psd]

psd = [str(round(x,4)) for x in psd]

with open(fp, 'a', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',',
                            quotechar=' ', quoting=csv.QUOTE_MINIMAL)
    writer.writerow([""])
    writer.writerow(["The next 8 lines contain the log of the first, these are the valued need to be input in the arduino program with rev > 0.2.0"])
    for i in range(8):
        if i < 7:
            writer.writerow(psd[16*i:16*(i+1)] + [""])
        else:
            writer.writerow(psd[16*i:16*(i+1)])
       