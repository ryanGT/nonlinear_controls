from matplotlib.pyplot import *
from scipy import *
import numpy
from numpy.linalg.linalg import eigvals
import control

import os, glob

from top_secret_model import A, B, C, D

sys = control.ss(A,B,C,D)

data = loadtxt('open_loop_pulse_test.csv', delimiter=',')

t = data[:,0]
dt = t[1]-t[0]
u = data[:,1]
th0 = data[:,-2]
th1 = data[:,-1]

figure(1)
clf()
plot(t,u,t,th0,t,th1)

tout, ysim, xsim = control.forced_response(sys, t, u, hmax=0.5*dt)

plot(t,ysim.T)


sys_d = control.matlab.c2d(sys, dt)

Adig = sys_d.A
Bdig = sys_d.B


Q = dot(C.T,C)

#set R to weight the penalty on input effort
# try changing R and watching the closed-loop pole locations change
R = 0.001
K, S, E = control.lqr(A, B, Q, R)

# check the closed-loop poles:
A_cl = A - dot(B,K)
eigs_cl = eigvals(A_cl)
print('closed-loop eigenvalues: ' + str(eigs_cl))

# run a continuous time closed-loop simulation
sys_cl = control.ss(A_cl,B,C,[[0],[0]])
y_cl,tout,x_cl = control.lsim(sys_cl, u, t)

# The next big step from a control implmentation stand point is how to
# do the feedback control digitally in real-time.  We need to first
# map the closed-loop pole locations from the continuous s domain to
# the discrete z domain using z = e^(s*dt)
z_cl = exp(eigs_cl*dt)
print('z_cl = ' + str(z_cl))
# then you Ackermann's formula to find the digital gains that put the
# closed-loop poles at those locations
Kdig = control.acker(Adig, Bdig, z_cl)
# verify the location of the closed-loop digital poles
A_cldig = Adig - dot(Bdig, Kdig)
z_poles_test = eigvals(A_cldig)
print('z_poles_test = ' + str(z_poles_test))

# Observer Design
# ==========================

# We can use LQR to design the observer if we use :inline:`A.T` and
# :inline:`C.T` instead of :inline:`A` and :inline:`B` in the
# calculations.  :inline:`Vo` and :inline:`Wo` weight the relative
# importance of sensor noise and noise on the system input.

# change the ratio of Vo/Wo to see how the observer poles move around
Vo = 1.0
Wo = 0.01

G = B#using G for clarity in comparing to most LQR notes
Qo = dot(G,dot(Vo,G.T))
C_calc = atleast_2d(C[1,:])#We will do LQR for just one output, the second one
Ko, So, Eo = control.lqr(A.T, C_calc.T, Qo, Wo)
L = Ko.T
ALC = A - dot(L,C_calc)

eigs_alc = eigvals(ALC)

# Now that we have the continuous time L, we need to find the corresponding
# digital gains:
z_mapped_obs = exp(eigs_alc*dt)
LT_dig_acker = control.acker(Adig.T,C_calc.T,z_mapped_obs)
Ldig = LT_dig_acker.T
L_dig_eigs = eigvals(Adig-dot(Ldig, C_calc))

Cdig = C_calc

run_test = 1

if run_test:
    import ss_qube_test
    print('running the test')
    data = ss_qube_test.run_dig_ss_test(Adig, Bdig, Cdig, \
                                        Kdig, Ldig, \
                                        T=10.0, dt=1.0/150, \
                                        amp=0, \
                                        open_loop=1)


    def save_data():
        #provide filename extension if there isn't one
        csvfiles = glob.glob('*.csv')
        print('Existing data files:')
        csvfiles.sort()
        for csv in csvfiles:
            print(csv)
        print('='*20)
        print('')
        filename = raw_input('filename: ')
        fno, ext = os.path.splitext(filename)
        ext = '.csv'
        filename = fno + ext
        if os.path.exists(filename):
            print('%s already exists, overwrite?' % filename)
            ow = raw_input('(y/n): ')
            if ow.lower()[0] != 'y':
                return

        labels = ['#t', 'u', 'v', 'theta0_vect', 'theta1_vect', 'th1_obs']

        data_str = data.astype('S30')
        data_out = numpy.append([labels],data_str, axis=0)
        
        savetxt(filename, data_out, delimiter=',',fmt='%s')
    
show()
