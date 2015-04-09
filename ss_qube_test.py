from matplotlib.pyplot import *
from numpy import *
import numpy, time

#import control
import control_utils

import time, copy, os

import serial_utils
#reload(serial_utils)


mymax = 70
def mysat(vin):
    if vin > mymax:
        return mymax
    elif vin < -mymax:
        return -mymax
    else:
        return vin



def run_dig_ss_test(Adig, Bdig, Cdig, \
                    Kdig, Ldig, \
                    T=3.0, dt=0.01, \
                    amp=50, \
                    open_loop=False, \
                    ):
    from myserial import ser

    ser.flushInput()
    ser.flushOutput()


    t = arange(0,T,dt)
    N = len(t)

    u = zeros(N,dtype=int)
    u[10:30] = amp

    nvect = zeros(N,dtype=int)
    vecho = zeros_like(nvect)
    theta0_vect = zeros_like(nvect)
    theta1_vect = zeros_like(nvect)
    v = zeros_like(nvect)

    n = len(Adig)

    x_hat = zeros((n,1))
    x_hat_mat = zeros((n,N))

    th0_obs = zeros(N)
    th1_obs = zeros(N)

    serial_utils.WriteByte(ser, 2)#start new test
    
    for i in range(N):
        if open_loop:
            v_float = u[i]
        else:
            v_float = -dot(Kdig, x_hat) + u[i]
            
        v_temp = int(v_float)
        v[i] = mysat(v_temp)

        th0_exp = theta0_vect[i-1]
        th1_exp = theta1_vect[i-1]#y_exp

        y_diff_th1 = th1_exp - dot(Cdig,x_hat)#y-y_hat
        obs1_part = dot(Ldig,squeeze(y_diff_th1))
        #x_hat = dot(Adig,x_hat) + Bdig*v[i] + dot(Ldig,squeeze(y_diff))
        x_hat = dot(Adig,x_hat) + Bdig*v[i] + obs1_part
        x_hat_mat[:,i] = squeeze(x_hat)
        th1_obs[i] = squeeze(dot(Cdig, x_hat))

        serial_utils.WriteByte(ser, 1)#new n and voltage are coming
        serial_utils.WriteInt(ser, i)
        serial_utils.WriteInt(ser, v[i])

        nvect[i] = serial_utils.Read_Two_Bytes(ser)
        vecho[i] = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
        theta0_vect[i] = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
        theta1_vect[i] = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
        nl_check = serial_utils.Read_Byte(ser)
        assert nl_check == 10, "newline problem"


    time.sleep(0.1)
    serial_utils.WriteByte(ser, 3)#stop test
    time.sleep(0.1)
    serial_utils.WriteByte(ser, 3)#stop test

    #serial_utils.Close_Serial(ser)


    figure(1)
    clf()
    plot(nvect, v, label='$v$')
    plot(nvect, theta0_vect, label='$\\theta_{0exp}$')
    plot(nvect, theta1_vect, label='$\\theta_{1exp}$')

    plot(nvect, th1_obs, label='$\\theta_{1obs}$')

    legend(loc=4)

    t = dt*nvect

    data = array([t, u, v, theta0_vect, theta1_vect, th1_obs]).T

    return data

    

def save_data(filename, datain):
    #provide filename extension if there isn't one
    fno, ext = os.path.splitext(filename)
    if not ext:
        ext = '.csv'
    filename = fno + ext

    labels = ['#t','u','v','theta0','theta1','theta1_obs']

    data_str = datain.astype(str)
    data_out = numpy.append([labels],data_str, axis=0)

    savetxt(filename, data_out, delimiter=',',fmt='%s')
    

show()
