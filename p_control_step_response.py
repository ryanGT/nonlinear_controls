from matplotlib.pyplot import *
from scipy import *
import numpy, time

import control, control_utils, bode_utils

import time, copy, os

import serial_utils
#reload(serial_utils)

from myserial import ser

ser.flushInput()
ser.flushOutput()


#time.sleep(0.1)

dt = 1.0/150#<---------- is this true for your choice of OCR1A?


## amp = 30
## u, t = control_utils.create_swept_sine_signal(deadtime=1.0, \
##                                               dt=dt)
## u *= amp

T = 3.0
t = arange(0,T,dt)

kp = 0.1

N = len(t)
u = zeros(N,dtype=int)
amp = 30
u[50:=amp]

nvect = zeros(N,dtype=int)
theta_vect = zeros_like(nvect)
v = zeros_like(nvect)
v_echo = zeros_like(nvect)
e = zeros_like(nvect)

serial_utils.WriteByte(ser, 2)#start new test
              
for i in range(N):
    e[i] = u[i] - theta_vect[i-1]
    v_float = kp*e[i]
    v[i] = int(v_float)
    
    serial_utils.WriteByte(ser, 1)#new n and voltage are coming
    serial_utils.WriteInt(ser, i)
    serial_utils.WriteInt(ser, v[i])

    nvect[i] = serial_utils.Read_Two_Bytes(ser)
    v_echo[i] = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
    theta_vect[i] = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
    nl_check = serial_utils.Read_Byte(ser)
    assert nl_check == 10, "newline problem"


time.sleep(0.1)
serial_utils.WriteByte(ser, 3)#stop test
time.sleep(0.1)
serial_utils.WriteByte(ser, 3)#stop test

#serial_utils.Close_Serial(ser)


figure(1)
clf()
plot(nvect, v)
plot(nvect, theta_vect)


t = dt*nvect

data = array([t, v, theta_vect]).T


Gjw = fft(theta_vect)/fft(v)
freq = control_utils.create_freq_vect(t)
bode_utils.bode_plot2(freq, Gjw, fignum=2)



#ylim([-10,100])

def save_data(filename, datain):
    #provide filename extension if there isn't one
    fno, ext = os.path.splitext(filename)
    if not ext:
        ext = '.csv'
    filename = fno + ext

    labels = ['#t','v','theta']

    data_str = datain.astype(str)
    data_out = numpy.append([labels],data_str, axis=0)

    savetxt(filename, data_out, delimiter=',')
    

show()
