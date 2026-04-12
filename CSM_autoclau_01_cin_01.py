import numpy as np
import matplotlib.pyplot as plt

# =========================
# FUNCIÓ
# =========================
def parametres_cicle(t, x, a_max):
    tb = t/2 - np.sqrt(a_max**2 * t**2 - 4*a_max*x) / (2*a_max)
    xb = 1/2 * a_max * tb**2
    tc = t - 2*tb
    xc = x - 2*xb
    v_max = a_max * tb
    return tb, xb, tc, xc, v_max

# DADES
# =========================
l_e = 150
l_t = 900
l_eix = 1000

r = 40
i = 36

t0 = 1

t1 = 3.0
t2 = 2.0
t3 = 2.5
t4 = 3.0

a = 1000

t_inc = 0.01

# DESPLAÇAMENTS
# =========================
x1 = l_e * 5
x2 = x1 - l_e
x3 = l_e * 5
x4 = l_t

# CICLES
# =========================
tb1, xb1, tc1, xc1, v1 = parametres_cicle(t1, x1, a)
tb2, xb2, tc2, xc2, v2 = parametres_cicle(t2, x2, a)
tb3, xb3, tc3, xc3, v3 = parametres_cicle(t3, x3, a)
tb4, xb4, tc4, xc4, v4 = parametres_cicle(t4, x4, a)

omega_max = max(abs(np.array([v1, v2, v3, v4]))) * i / r
n_max = omega_max * 60 / (2*np.pi)
eps_max = a * i / r


# TEMPS
ti = np.zeros(14)

ti[0]  = tb1
ti[1]  = ti[0] + tc1
ti[2]  = ti[1] + tb1
ti[3]  = ti[2] + t0
ti[4]  = ti[3] + tb2
ti[5]  = ti[4] + tc2
ti[6]  = ti[5] + tb2
ti[7]  = ti[6] + t0
ti[8]  = ti[7] + tb3
ti[9]  = ti[8] + tc3
ti[10] = ti[9] + tb3
ti[11] = ti[10] + tb4
ti[12] = ti[11] + tc4
ti[13] = ti[12] + tb4

t_total = ti[13]


# POSICIÓ
xi = np.zeros(14)

xi[0]  = + 1/2*a*ti[0]**2
xi[1]  = xi[0] + v1*(ti[1]-ti[0])
xi[2]  = xi[1] + v1*(ti[2]-ti[1]) - 1/2*a*(ti[2]-ti[1])**2
xi[3]  = xi[2]

xi[4]  = xi[3] - 1/2*a*(ti[4]-ti[3])**2
xi[5]  = xi[4] - v2*(ti[5]-ti[4])
xi[6]  = xi[5] - v2*(ti[6]-ti[5]) + 1/2*a*(ti[6]-ti[5])**2
xi[7]  = xi[6]

xi[8]  = xi[7] + 1/2*a*(ti[8]-ti[7])**2
xi[9]  = xi[8] + v3*(ti[9]-ti[8])
xi[10] = xi[9] + v3*(ti[10]-ti[9]) - 1/2*a*(ti[10]-ti[9])**2
xi[11] = xi[10] - 1/2*a*(ti[11]-ti[10])**2
xi[12] = xi[11] - v4*(ti[12]-ti[11])
xi[13] = xi[12] - v4*(ti[13]-ti[12]) + 1/2*a*(ti[13]-ti[12])**2

x_total = xi[13]


t_ = np.arange(0, t_total, t_inc)
x_ = np.zeros(len(t_))
v_ = np.zeros(len(t_))
a_ = np.zeros(len(t_))

for ii in range(len(t_)):

    if t_[ii] <= ti[0]:
        x_[ii] = + 1/2*a*t_[ii]**2
        v_[ii] = a*t_[ii]
        a_[ii] = a

    elif t_[ii] <= ti[1]:
        x_[ii] = xi[0] + v1*(t_[ii]-ti[0])
        v_[ii] = v1
        a_[ii] = 0

    elif t_[ii] <= ti[2]:
        x_[ii] = xi[1] + v1*(t_[ii]-ti[1]) - 1/2*a*(t_[ii]-ti[1])**2
        v_[ii] = v1 - a*(t_[ii]-ti[1])
        a_[ii] = -a

    elif t_[ii] <= ti[3]:
        x_[ii] = xi[2]
        v_[ii] = 0
        a_[ii] = 0

    elif t_[ii] <= ti[4]:
        x_[ii] = xi[3] - 1/2*a*(t_[ii]-ti[3])**2
        v_[ii] = -a*(t_[ii]-ti[3])
        a_[ii] = -a

    elif t_[ii] <= ti[5]:
        x_[ii] = xi[4] - v2*(t_[ii]-ti[4])
        v_[ii] = -v2
        a_[ii] = 0

    elif t_[ii] <= ti[6]:
        x_[ii] = xi[5] - v2*(t_[ii]-ti[5]) + 1/2*a*(t_[ii]-ti[5])**2
        v_[ii] = -v2 + a*(t_[ii]-ti[5])
        a_[ii] = a

    elif t_[ii] <= ti[7]:
        x_[ii] = xi[6]
        v_[ii] = 0
        a_[ii] = 0

    elif t_[ii] <= ti[8]:
        x_[ii] = xi[7] + 1/2*a*(t_[ii]-ti[7])**2
        v_[ii] = a*(t_[ii]-ti[7])
        a_[ii] = a

    elif t_[ii] <= ti[9]:
        x_[ii] = xi[8] + v3*(t_[ii]-ti[8])
        v_[ii] = v3
        a_[ii] = 0

    elif t_[ii] <= ti[10]:
        x_[ii] = xi[9] + v3*(t_[ii]-ti[9]) - 1/2*a*(t_[ii]-ti[9])**2
        v_[ii] = v3 - a*(t_[ii]-ti[9])
        a_[ii] = -a

    elif t_[ii] <= ti[11]:
        x_[ii] = xi[10] - 1/2*a*(t_[ii]-ti[10])**2
        v_[ii] = -a*(t_[ii]-ti[10])
        a_[ii] = -a

    elif t_[ii] <= ti[12]:
        x_[ii] = xi[11] - v4*(t_[ii]-ti[11])
        v_[ii] = -v4
        a_[ii] = 0

    else:
        x_[ii] = xi[12] - v4*(t_[ii]-ti[12]) + 1/2*a*(t_[ii]-ti[12])**2
        v_[ii] = -v4 + a*(t_[ii]-ti[12])
        a_[ii] = a


# =========================
# GRÀFIQUES POSICIÓ, VELOCITAT i ACCELERACIÓ
# =========================

# Moviment carro guia
plt.figure()

plt.subplot(3,1,1)
plt.plot(t_, x_)
plt.ylabel(r"$u$ [mm]")
plt.xlim([0, t_total])

plt.subplot(3,1,2)
plt.plot(t_, v_)
plt.ylabel(r"$v$ [mm/s]")
plt.xlim([0, t_total])

plt.subplot(3,1,3)
plt.plot(t_, a_)
plt.ylabel(r"$a$ [mm/s²]")
plt.xlabel(r"$t$ [s]")
plt.xlim([0, t_total])

plt.tight_layout()
plt.show()

# Eix motor
plt.figure()

plt.subplot(2,1,1)
plt.plot(t_, v_*i/r*(30/np.pi))
plt.ylabel(r"$n_m$ [rpm]")
plt.xlim([0, t_total])

plt.subplot(2,1,2)
plt.plot(t_, a_*i/r)
plt.ylabel(r"$\epsilon_m$ [rad/s²]")
plt.xlabel(r"$t$ [s]")
plt.xlim([0, t_total])

plt.tight_layout()
plt.show()