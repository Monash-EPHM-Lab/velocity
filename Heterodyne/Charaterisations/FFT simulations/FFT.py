import numpy as np
import matplotlib.pyplot as plt
import math
import random
from operator import add
from scipy.fft import fft, ifft
from scipy import ndimage


freqin = [0]*256;

for i in range(256):

    if i < 128:
        j = i
    else:
        j = 256-i
    
    x = (j-25)/2
    
    freqin[i] = 2.71**(-x*x)


sins = ifft(freqin)

print(np.sqrt(np.mean(sins**2)))

def gen(i):
    global sins
    
    val = 0
    
    val += sins[i]*7
    
    val += random.uniform(0, 1)

    val *= 4
    
    val = round(val,0)
    
    return val

fvals = [0]*256;


for av in range(4):

    vals = []

    for i in range(256):
        vals.append(gen(i))



    fvals = list(map(add, fvals,fft(vals)))



plt.subplot(2, 1, 2)


plt.plot(np.abs(fvals[1:127]))
plt.xlabel("Frequency")
plt.title("FFT")

plt.subplot(2, 1, 1)

plt.plot(vals)
plt.xlabel("Time")
plt.title("FFT - input sample")

plt.show()
