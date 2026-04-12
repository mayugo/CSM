import numpy as np
import matplotlib.pyplot as plt

# %%
# ======================
# DADES
# ======================
t_total = 8.5      # s, temps total de cicle
a_max = 200        # mm/s^2, acceleració màxima

rP = 80 / 2        # mm, radi politja
i = 66

# %%
# ======================
# Resolució: Métode 1, marxa-moviment-parada
# ======================
# S'imposa un temps de treball i una velocitat de treball a l'anada
t_treball = 4       # s, temps de treball
v_treball = 100     # mm/s, velocitat de treball

# Paràmetres del cicle d'anada (ta)
tb_a = v_treball / a_max
xb_a = 0.5 * a_max * tb_a**2

ta = 2 * tb_a + t_treball
xa = 2 * xb_a + t_treball * v_treball

# Paràmetres del cicle de tornada (tt)
tt = t_total - ta

tb_t = (tt/2) - np.sqrt(a_max**2 * tt**2 - 4 * a_max * xa) / (2 * a_max)
xb_t = 0.5 * a_max * tb_t**2

v_max = tb_t * a_max

# En el motor
omega_max = v_max * i / rP
omega_treball = v_treball * i / rP

n_max = omega_max * 60 / (2*np.pi)
n_treball = omega_treball * 60 / (2*np.pi)

# % Definir trams de moviment: motion control
ti = np.zeros(6)
ti[0] = tb_a
ti[1] = ti[0] + t_treball
ti[2] = ti[1] + tb_a
ti[3] = ti[2] + tb_t
ti[4] = ti[3] + tt - 2*tb_t
ti[5] = ti[4] + tb_t

xi = np.zeros(6)
xi[0] = 0.5 * a_max * ti[0]**2
xi[1] = xi[0] + v_treball * (ti[1] - ti[0])
xi[2] = xi[1] + v_treball * (ti[2] - ti[1]) - 0.5 * a_max * (ti[2] - ti[1])**2
xi[3] = xi[2] - 0.5 * a_max * (ti[3] - ti[2])**2
xi[4] = xi[3] - v_max * (ti[4] - ti[3])
xi[5] = xi[4] - v_max * (ti[5] - ti[4]) + 0.5 * a_max * (ti[5] - ti[4])**2

# % Obtenir la trajectoria posicio, velocitat i acceleració
t_ = np.arange(0, t_total, 0.05)
x_ = np.zeros_like(t_)
v_ = np.zeros_like(t_)
a_ = np.zeros_like(t_)

for k, t in enumerate(t_):
    if t <= ti[0]:
        x_[k] = 0.5 * a_max * t**2
        v_[k] = a_max * t
        a_[k] = a_max

    elif t <= ti[1]:
        x_[k] = xi[0] + v_treball * (t - ti[0])
        v_[k] = v_treball

    elif t <= ti[2]:
        x_[k] = xi[1] + v_treball*(t - ti[1]) - 0.5*a_max*(t - ti[1])**2
        v_[k] = v_treball - a_max*(t - ti[1])
        a_[k] = -a_max

    elif t <= ti[3]:
        x_[k] = xi[2] - 0.5*a_max*(t - ti[2])**2
        v_[k] = -a_max*(t - ti[2])
        a_[k] = -a_max

    elif t <= ti[4]:
        x_[k] = xi[3] - v_max*(t - ti[3])
        v_[k] = -v_max

    else:
        x_[k] = xi[4] - v_max*(t - ti[4]) + 0.5*a_max*(t - ti[4])**2
        v_[k] = -v_max + a_max*(t - ti[4])
        a_[k] = a_max

# %%
#  Representar poció, velocitat i acceleració
plt.figure(figsize=(8,6))

plt.subplot(3,1,1)
plt.plot(t_, x_)
plt.ylabel(r"$u$ [mm]")

plt.subplot(3,1,2)
plt.plot(t_, v_)
plt.ylabel(r"$v$ [mm/s]")

plt.subplot(3,1,3)
plt.plot(t_, a_)
plt.ylabel(r"$a$ [mm/s²]")
plt.xlabel(r"$t$ [s]")

plt.tight_layout()
plt.show()

plt.figure(figsize=(8,3))
plt.plot(t_, v_ * i / rP)
plt.ylabel(r"$\omega_m$ [rad/s]")
plt.xlabel("$t$ [s]")
plt.show()

# %%
# =========================================================
# Resolució: Métode 2, definir trajectòria punt a punt
# =========================================================
# S'imposa arranc inicial, llavors un moviment de treball i un de tornada
# definint 4 punts
td12 = 0.4
td23 = 4.9
td34 = t_total - (td12 + td23)

x1, x2, x3, x4 = 0, 10, 500, 0    # mm, 

## Paràmetres trams
# % tram de 1 a 2 (inicial)
a1 = np.sign(x2-x1) * a_max
t1 = td12 - np.sqrt(td12**2 - 2*(x2-x1)/a1)
v12 = (x2-x1)/(td12 - 0.5*t1)

# % tram de 3 a 4 (final)
a4 = np.sign(x3-x4) * a_max
t4 = td34 - np.sqrt(td34**2 + 2*(x4-x3)/a4)
v34 = (x4-x3)/(td34 - 0.5*t4)

# % VELOCITATS: tram de 2 a 3
v23 = (x3-x2)/td23

# % ACCELERACIONS: tram 2
a2 = np.sign(v23-v12) * a_max
a3 = np.sign(v34-v23) * a_max

# % intervals de temps
t2 = (v23 - v12)/a2
t3 = (v34 - v23)/a3
t12 = td12 - t1 - 0.5*t2
t23 = td23 - 0.5*(t2 + t3)
t34 = td34 - t4 - 0.5*t3
ttotal2=t1+t2+t3+t4+t12+t23+t34

# % intervals de POSICIÓ
xi1 = 0 *t1  + 1/2*a1*t1**2;
xi2 = v12*t2 + 1/2*a2*t2**2;
xi3 = v23*t3 + 1/2*a3*t3**2;
xi4 = v34*t4 + 1/2*a4*t4**2;
x12 = v12*t12;
x23 = v23*t23;
x34 = v34*t34;
xtotal2 = xi1 + xi2+xi3+xi4+x12+x23+x34

print(f"Tram 1 , MUA, t_1 = {t1:10.3f} s , x_1 = {xi1:10.3f} mm, a_1 = {a1:10.0f} mm/s^2")
print(f"Tram 12, MU, t_12 = {t12:10.3f} s , x_12 = {x12:10.3f} mm, v_12 = {v12:10.0f} mm/s")
print(f"Tram 2 , MUA, t_2 = {t2:10.3f} s , x_2 = {xi2:10.3f} mm, a_2 = {a2:10.0f} mm/s^2")
print(f"Tram 23, MU, t_23 = {t23:10.3f} s , x_23 = {x23:10.3f} mm, v_23 = {v23:10.0f} mm/s")
print(f"Tram 3 , MUA, t_3 = {t3:10.3f} s , x_3 = {xi3:10.3f} mm, a_3 = {a3:10.0f} mm/s^2")
print(f"Tram 34, MU, t_34 = {t34:10.3f} s , x_34 = {x34:10.3f} mm, v_34 = {v34:10.0f} mm/s")
print(f"Tram 4 , MUA, t_4 = {t4:10.3f} s , x_4 = {xi4:10.3f} mm, a_4 = {a4:10.0f} mm/s^2")
print(" ")
print(f"Temps total = {ttotal2:10.3f} s , x total = {abs(xtotal2):10.3f} mm")

# %%
ti2 = np.zeros(7)
ti2[0] = t1
ti2[1] = ti2[0] + t12
ti2[2] = ti2[1] + t2
ti2[3] = ti2[2] + t23
ti2[4] = ti2[3] + t3
ti2[5] = ti2[4] + t34
ti2[6] = ti2[5] + t4

xi_2 = np.zeros(7)
xi_2[0] = xi1
xi_2[1] = xi_2[0] + x12
xi_2[2] = xi_2[1] + xi2
xi_2[3] = xi_2[2] + x23
xi_2[4] = xi_2[3] + xi3
xi_2[5] = xi_2[4] + x34
xi_2[6] = xi_2[5] + xi4

# % Obtenir la trajectoria posicio, velocitat i acceleració
t_ = np.arange(0, t_total, 0.05)
x2_ = np.zeros_like(t_)
v2_ = np.zeros_like(t_)
a2_ = np.zeros_like(t_)

for k, t in enumerate(t_):
    if t <= ti2[0]:
        x2_[k] = 0 + 0.5*a1*t**2
        v2_[k] = a1*t
        a2_[k] = a1
    elif t <= ti2[1]:
        x2_[k] = xi_2[0] + v12*(t - ti2[0])
        v2_[k] = v12
        a2_[k] = 0
    elif t <= ti2[2]:
        x2_[k] = xi_2[1] + v12*(t - ti2[1]) + 0.5*a2*(t - ti2[1])**2
        v2_[k] = v12 + a2*(t - ti2[1])
        a2_[k] = a2
    elif t <= ti2[3]:
        x2_[k] = xi_2[2] + v23*(t - ti2[2])
        v2_[k] = v23
        a2_[k] = 0
    elif t <= ti2[4]:
        x2_[k] = xi_2[3] + v23*(t - ti2[3]) + 0.5*a3*(t - ti2[3])**2
        v2_[k] = v23 + a3*(t - ti2[3])
        a2_[k] = a3
    elif t <= ti2[5]:
        x2_[k] = xi_2[4] + v34*(t - ti2[4])
        v2_[k] = v34
        a2_[k] = 0
    else:
        x2_[k] = xi_2[5] + v34*(t - ti2[5]) + 0.5*a4*(t - ti2[5])**2
        v2_[k] = v34 + a4*(t - ti2[5])
        a2_[k] = a4

# %%
# % Representar poció, velocitat i acceleració
plt.figure(figsize=(8,6))
plt.subplot(3,1,1)
plt.plot(t_, x2_)
plt.ylabel(r"$u$ [mm]")

plt.subplot(3,1,2)
plt.plot(t_, v2_)
plt.ylabel(r"$v$ [mm/s]")

plt.subplot(3,1,3)
plt.plot(t_, a2_)
plt.ylabel(r"$a$ [mm/s²]")
plt.xlabel(r"$t$ [s]")

plt.tight_layout()
plt.show()