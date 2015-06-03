__author__ = 'Timothy Hadwen'

import serial
from pylab import *
import _thread
import time
data1 = []
data2 = []

plt.show()
plt.ion()
plt.winter()

def serial_thread():
    count = 0
    ser = serial.Serial('/dev/tty.usbmodem1411', 115200, timeout=1)
    while(True):
        s = ser.readline()        # read up to ten bytes (timeout)
        if(s.decode("utf-8")[:4] == "FOUND"):
            print(s.decode("utf-8"))
            print("SOUND WAS FOUND")
        else:
            try:
                data_in = str.split(s.decode("utf-8"))
                #print(s)
                data1.append(int(data_in[0], 16))
                data2.append(int(data_in[1], 16))
                count += 1
            except:
                print("Connect Error")

        if len(data1) > 500:
            data1.pop(0)

        if len(data2) > 500:
            data2.pop(0)

_thread.start_new_thread(serial_thread, ())

# Wait for some data to appear before calculating fft
time.sleep(5)
while(True):
    #print(data[len(data)-10: len(data)])
    plt.subplot(4,1,1)
    plt.cla()
    plt.plot(data1)
    plt.xlim(0, 500)
    plt.draw()

    plt.subplot(4,1,2)
    n = len(data1)
    p = fft(data1) # take the fourier transform

    nUniquePts = math.ceil((n+1)/2.0)
    p = p[0:nUniquePts]
    p = abs(p)

    p = p / float(n) # scale by the number of points so that
             # the magnitude does not depend on the length
             # of the signal or on its sampling frequency
    p = p**2  # square it to get the power

    # multiply by two (see technical document for details)
    # odd nfft excludes Nyquist point
    if n % 2 > 0: # we've got odd number of points fft
        p[1:len(p)] = p[1:len(p)] * 2
    else:
        p[1:len(p) -1] = p[1:len(p) - 1] * 2 # we've got even number of points fft
    plt.cla()
    freqArray = arange(0, nUniquePts, 1.0) * (48000 / n);
    plt.plot(freqArray/1000, 20*log10(p))
    plt.xlabel('Frequency (kHz)')
    plt.ylabel('Power (dB)')

    plt.xlim(0, 20)
    plt.draw()

    # Plot the second microphone as well
    plt.subplot(4,1,3)
    plt.cla()
    plt.plot(data2)
    plt.xlim(0, 500)
    plt.draw()

    plt.subplot(4,1,4)
    n = len(data2)
    p = fft(data2) # take the fourier transform

    nUniquePts = math.ceil((n+1)/2.0)
    p = p[0:nUniquePts]
    p = abs(p)

    p = p / float(n) # scale by the number of points so that
             # the magnitude does not depend on the length
             # of the signal or on its sampling frequency
    p = p**2  # square it to get the power

    # multiply by two (see technical document for details)
    # odd nfft excludes Nyquist point
    if n % 2 > 0: # we've got odd number of points fft
        p[1:len(p)] = p[1:len(p)] * 2
    else:
        p[1:len(p) -1] = p[1:len(p) - 1] * 2 # we've got even number of points fft
    plt.cla()
    freqArray = arange(0, nUniquePts, 1.0) * (48000 / n);
    plt.plot(freqArray/1000, 20*log10(p))
    plt.xlabel('Frequency (kHz)')
    plt.ylabel('Power (dB)')

    plt.xlim(0, 20)
    plt.draw()

    time.sleep(0.01)