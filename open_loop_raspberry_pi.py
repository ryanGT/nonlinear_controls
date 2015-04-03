from matplotlib.pyplot import *
from numpy import *
import numpy, time

#import control
import control_utils

import time, copy, os

import serial_utils
#reload(serial_utils)

from myserial import ser

ser.flushInput()
ser.flushOutput()


#time.sleep(0.1)

dt = 1.0/150#<---------- is this true for your choice of OCR1A?


N = 150*3#10 second
amp = 50

u = zeros(N)
u[10:20] = amp

nvect = zeros(N,dtype=int)
theta0 = zeros_like(nvect)
theta1 = zeros_like(nvect)
v_echo = zeros_like(nvect)

serial_utils.WriteByte(ser, 2)#start new test


for i in range(N):
    serial_utils.WriteByte(ser, 1)#new n and voltage are coming
    serial_utils.WriteInt(ser, i)
    serial_utils.WriteInt(ser, u[i])

    nvect[i] = serial_utils.Read_Two_Bytes(ser)
    v_echo[i] = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
    theta0[i] = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
    theta1[i] = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
    nl_check = serial_utils.Read_Byte(ser)
    assert nl_check == 10, "newline problem"


time.sleep(0.1)
serial_utils.WriteByte(ser, 3)#stop test
time.sleep(0.1)
serial_utils.WriteByte(ser, 3)#stop test

#serial_utils.Close_Serial(ser)


figure(1)
clf()
plot(nvect, u)
plot(nvect, theta0)
plot(nvect, theta1)


t = dt*nvect

data = array([t, u, theta0, theta1]).T


#ylim([-10,100])

def save_data(filename, datain):
    #provide filename extension if there isn't one
    fno, ext = os.path.splitext(filename)
    if not ext:
        ext = '.csv'
    filename = fno + ext

    labels = ['#t','u','theta0','theta1']

    data_str = datain.astype('S30')
    data_out = numpy.append([labels],data_str, axis=0)

    savetxt(filename, data_out, delimiter=',',fmt='%s')
    

show()
