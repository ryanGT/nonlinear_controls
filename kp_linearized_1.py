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


test_case = 1

if test_case == 1:
    N = 150*10

    #v = arange(N)


    #amp = 50
    amp = 0
    v_case = 3

    if v_case == 1:
        v = zeros(N,dtype=int)
        v[10:30] = amp
    elif v_case == 2:
        v = ones(N,dtype=int)*amp
    elif v_case == 3:
        u, t, = control_utils.create_swept_sine_signal(dt=dt, \
                                                       tspan=5, \
                                                       fmax=10.0)
        v = (u*amp).astype(int)
        N = len(v)
        
elif test_case == 2:#swept_sine
    amp = 10.0
    u, t = control_utils.create_swept_sine_signal(deadtime=1.0, \
                                                  dt=dt)
    v_float = u*amp
    v = v_float.astype(int)
    N = len(t)


nvect = zeros(N,dtype=int)
theta0 = zeros_like(nvect)
theta1 = zeros_like(nvect)
v_echo = zeros_like(nvect)

serial_utils.WriteByte(ser, 2)#start new test


kp = 1.0

def kp_calc(theta, theta_d):
    e = theta_d-theta
    return kp*e

mymax = 70
def mysat(vin):
    if vin > mymax:
        return mymax
    elif vin < -mymax:
        return -mymax
    else:
        return vin

    
for i in range(N):

    if i == 0:
        v[i] = 0
    elif i == 1:
        theta_initial = theta1[0]
        if theta_initial > 800:
            theta_d = 1048
        elif theta_initial < -800:
            theta_d = -1048
        else:
            raise ValueError, "bad initial theta1"

    if i >=1:
        v_temp = kp_calc(theta1[i-1], theta_d)
        v[i] = mysat(v_temp)
        
    serial_utils.WriteByte(ser, 1)#new n and voltage are coming
    serial_utils.WriteInt(ser, i)
    serial_utils.WriteInt(ser, v[i])

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
plot(nvect, v)
plot(nvect, theta0)
plot(nvect, theta1)


t = dt*nvect

data = array([t, v, theta0, theta1]).T


#ylim([-10,100])

def save_data(filename, datain):
    #provide filename extension if there isn't one
    fno, ext = os.path.splitext(filename)
    if not ext:
        ext = '.csv'
    filename = fno + ext

    labels = ['#t','v','theta0','theta1']

    data_str = datain.astype(str)
    data_out = numpy.append([labels],data_str, axis=0)

    savetxt(filename, data_out, delimiter=',',fmt='%s')
    

show()
