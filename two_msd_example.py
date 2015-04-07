# 2DOF system explanation of digital state-space simulation
from matplotlib.pyplot import *
from scipy import *
import numpy
from numpy.linalg.linalg import eigvals
import control

m1 = 2.0
m2 = 3.0
k1 = 100.0
k2 = 75.0
b1 = 1.0
b2 = 1.5

# Creating the State-Space Model
# =========================================

# states: x1=x1, x2=x2, x3=x1_dot, x4=x2_dot
# [x1_dot, x2_dot, x3_dot, x4_dot].T  = [A]*[x]+[B]*F
A = array([[0,0,1,0], \
           [0,0,0,1], \
           [-(k2+k1)/m1, k2/m1, -(b2+b1)/m1, b2/m1], \
           [k2/m2, -k2/m2, b2/m2, -b2/m2], \
           ])

B = array([[0.0], \
           [0.0], \
           [0.0], \
           [1.0/m2]])

# set x1 and x2 as the measurable outputs
C = array([[1.0,0,0,0], \
           [0.0,1,0,0]])


# create the continuous time system
sys_c = control.ss(A,B,C,[[0],[0]])

# create the vectors to run a step response
T = 10.0
dt = 0.01
t = arange(0,T,dt)
u = zeros_like(t)
u[10:] = 1.0

# run a continuous time simulation
y,tout,x = control.lsim(sys_c, u, t)

figure(1)
clf()
plot(t,y)

xlabel('Time (sec.)')
ylabel('Position')
legend(['$x_1$','$x_2$'])

# Discretizing the State-Space Model
# ===========================================

# Alternatively, we can find the digital matrices and run the
# simulation within a for loop, like we will need to in real-time on
# the raspberry pi

# use the c2d function to convert the continuous system to a digital one
sys_d = control.matlab.c2d(sys_c, dt)

A_dig = sys_d.A
B_dig = sys_d.B
# note that C is unchanged by the c2d conversion
print('A_dig = ' + str(A_dig))
print('B_dig = ' + str(B_dig))

N = len(t)
x_dig = zeros((4,N))
y_dig = zeros((2,N))

def as_column(vect):
    """Convert a vector to a matrix and check if it is a row or
    column.  If it is a row, transpose it."""
    mat = atleast_2d(vect)
    nr,nc = mat.shape
    if nr < nc:
        return mat.T
    else:
        return mat
    
    
for i in range(N):
    x_prev = as_column(x_dig[:,i-1])
    x_next = dot(A_dig, x_prev) + B_dig*u[i]
    x_dig[:,i] = squeeze(x_next)
    y_dig[:,i] = squeeze(dot(C,x_next))


plot(t,y_dig.T,'--', linewidth=2)

legend(['$x_1$','$x_2$','$x_{1dig}$','$x_{2dig}$'])


# LQR Control Design
# ==========================

# set Q to penalize the states related to our output
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

figure(2)
clf()
plot(t, y_cl)


# The next big step from a control implmentation stand point is how to
# do the feedback control digitally in real-time.  We need to first
# map the closed-loop pole locations from the continuous s domain to
# the discrete z domain using z = e^(s*dt)
z_cl = exp(eigs_cl*dt)
print('z_cl = ' + str(z_cl))
# then you Ackermann's formula to find the digital gains that put the
# closed-loop poles at those locations
K_dig = control.acker(A_dig, B_dig, z_cl)
# verify the location of the closed-loop digital poles
A_cl_dig = A_dig - dot(B_dig, K_dig)
z_poles_test = eigvals(A_cl_dig)
print('z_poles_test = ' + str(z_poles_test))

# run a digital closed-loop simulation
x_dig_cl = zeros((4,N))
v_dig_cl = zeros(N)
y_dig_cl = zeros((2,N))

for i in range(N):
    x_prev = as_column(x_dig_cl[:,i-1])
    v_dig_cl[i] = -dot(K_dig, x_prev) + u[i]
    x_next = dot(A_dig, x_prev) + B_dig*v_dig_cl[i]
    x_dig_cl[:,i] = squeeze(x_next)
    y_dig_cl[:,i] = squeeze(dot(C,x_next))


plot(t,y_dig_cl.T,'--', linewidth=2)


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
LT_dig_acker = control.acker(A_dig.T,C_calc.T,z_mapped_obs)
L_dig = LT_dig_acker.T
L_dig_eigs = eigvals(A_dig-dot(L_dig, C_calc))


# run a digital closed-loop simulation with the observer
x_hat_cl = zeros((4,N))
v_dig_obs = zeros(N)
y_dig_obs = zeros((2,N))
noise_scale = 0.0005


# assume we have a model that is slightly off from the actual system
A_model = zeros_like(A_dig)
nr,nc = A_dig.shape
for i in range(nr):
    for j in range(nc):
        # modeling will be a random number up to 1% of the current
        # entry in A_dig
        ent = A_dig[i,j]
        cur_error = 0.001*rand()*abs(ent)
        A_model[i,j] = ent + cur_error 

for i in range(N):
    x_prev = as_column(x_hat_cl[:,i-1])
    y_prev = y_dig_obs[1,i-1]#<-- chage this to use the experimental
                            #system when running an actual test
    v_dig_obs[i] = -dot(K_dig, x_prev) + u[i]
    x_next = dot(A_model, x_prev) + B_dig*v_dig_obs[i] + \
               L_dig*(y_prev - dot(C_calc, x_prev))
    x_hat_cl[:,i] = squeeze(x_next)
    y_dig_obs[:,i] = squeeze(dot(C,x_next))
    y_dig_obs[0,i] += noise_scale*rand()#add some fake sensor noise
    y_dig_obs[1,i] += noise_scale*rand()#add some fake sensor noise


plot(t,y_dig_obs.T,'-.', linewidth=2)

show()
