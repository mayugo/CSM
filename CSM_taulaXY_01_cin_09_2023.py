import numpy as np
import matplotlib.pyplot as plt

# =========================
# DADES
# =========================
n_max = 6460  # rpm
n_nom = 5060  # rpm
P_nom = 20    # W

i = 66

rG = 70e-3
rP = 40e-3

npuls = 4
Xcomp = 1

x_f = 200e-3
y_f = 150e-3

eps_max = 1000
t_total = 2.4


# =========================
# CINEMÀTICA INVERSA
# =========================
i_t = i / rG

theta1_f = i_t * (x_f + y_f)
theta2_f = i_t * (-x_f)

# =========================
# DETERMINACIÓ MOVIMENT CRÍTIC
# =========================
if abs(theta1_f) > abs(theta2_f):

    ddtheta1 = eps_max * np.sign(theta1_f)

    t_b = t_total/2 - np.sqrt(ddtheta1**2 * t_total**2 - 4*abs(ddtheta1)*abs(theta1_f)) / (2*abs(ddtheta1))
    dtheta1 = t_b * ddtheta1

    ddtheta2 = theta2_f / (t_b*t_total - t_b**2)
    dtheta2 = t_b * ddtheta2

else:

    ddtheta2 = eps_max * np.sign(theta2_f)

    t_b = t_total/2 - np.sqrt(ddtheta2**2 * t_total**2 - 4*abs(ddtheta2)*abs(theta2_f)) / (2*abs(ddtheta2))
    dtheta2 = t_b * ddtheta2

    ddtheta1 = theta1_f / (t_b*t_total - t_b**2)
    dtheta1 = t_b * ddtheta1


# =========================
# TEMPS
# =========================
t_abs1 = np.zeros(3)

t_abs1[0] = t_b
t_abs1[1] = t_abs1[0] + t_total - 2*t_b
t_abs1[2] = t_abs1[1] + t_b


# =========================
# POSICIONS FINALS TRAMS
# =========================
theta1_a = np.zeros(3)
theta2_a = np.zeros(3)

theta1_a[0] = 0.5 * ddtheta1 * t_abs1[0]**2
theta1_a[1] = theta1_a[0] + dtheta1*(t_abs1[1]-t_abs1[0])
theta1_a[2] = theta1_a[1] + dtheta1*(t_abs1[2]-t_abs1[1]) - 0.5*ddtheta1*(t_abs1[2]-t_abs1[1])**2

theta2_a[0] = 0.5 * ddtheta2 * t_abs1[0]**2
theta2_a[1] = theta2_a[0] + dtheta2*(t_abs1[1]-t_abs1[0])
theta2_a[2] = theta2_a[1] + dtheta2*(t_abs1[2]-t_abs1[1]) - 0.5*ddtheta2*(t_abs1[2]-t_abs1[1])**2


# =========================
# DISPLAY (similar MATLAB disp)
# =========================
print("RESULTATS DE L'EXERCICI")
print()

print("Motor 1")
print(f"t_b = {t_b:.3f} s, theta1_1 = {theta1_a[0]:.3f} rad, eps1 = {ddtheta1:.3f} rad/s2")
print(f"omega1 = {dtheta1:.3f} rad/s")
print(f"theta1 total = {abs(theta1_a[2]):.3f} rad")
print()

print("Motor 2")
print(f"t_b = {t_b:.3f} s, theta2_1 = {theta2_a[0]:.3f} rad, eps2 = {ddtheta2:.3f} rad/s2")
print(f"omega2 = {dtheta2:.3f} rad/s")
print(f"theta2 total = {abs(theta2_a[2]):.3f} rad")


# =========================
# TRAJECTÒRIA DISCRETITZADA
# =========================
t_ = np.arange(0, t_total, 0.005)

theta1_ = np.zeros(len(t_))
dtheta1_ = np.zeros(len(t_))
ddtheta1_ = np.zeros(len(t_))

theta2_ = np.zeros(len(t_))
dtheta2_ = np.zeros(len(t_))
ddtheta2_ = np.zeros(len(t_))


# =========================
# BUCLE MOTOR 1 (FIDEL MATLAB)
# =========================
for ii in range(len(t_)):

    if t_[ii] <= t_abs1[0]:
        theta1_[ii] = 0.5*ddtheta1*t_[ii]**2
        dtheta1_[ii] = ddtheta1*t_[ii]
        ddtheta1_[ii] = ddtheta1

    elif t_[ii] <= t_abs1[1]:
        theta1_[ii] = theta1_a[0] + dtheta1*(t_[ii]-t_abs1[0])
        dtheta1_[ii] = dtheta1
        ddtheta1_[ii] = 0

    else:
        theta1_[ii] = theta1_a[1] + dtheta1*(t_[ii]-t_abs1[1]) - 0.5*ddtheta1*(t_[ii]-t_abs1[1])**2
        dtheta1_[ii] = dtheta1 - ddtheta1*(t_[ii]-t_abs1[1])
        ddtheta1_[ii] = -ddtheta1


# =========================
# BUCLE MOTOR 2 (FIDEL MATLAB)
# =========================
for ii in range(len(t_)):

    if t_[ii] <= t_abs1[0]:
        theta2_[ii] = 0.5*ddtheta2*t_[ii]**2
        dtheta2_[ii] = ddtheta2*t_[ii]
        ddtheta2_[ii] = ddtheta2

    elif t_[ii] <= t_abs1[1]:
        theta2_[ii] = theta2_a[0] + dtheta2*(t_[ii]-t_abs1[0])
        dtheta2_[ii] = dtheta2
        ddtheta2_[ii] = 0

    else:
        theta2_[ii] = theta2_a[1] + dtheta2*(t_[ii]-t_abs1[1]) - 0.5*ddtheta2*(t_[ii]-t_abs1[1])**2
        dtheta2_[ii] = dtheta2 - ddtheta2*(t_[ii]-t_abs1[1])
        ddtheta2_[ii] = -ddtheta2


# =========================
# GRÀFIQUES MOTORS
# =========================
plt.figure()

plt.subplot(3,1,1)
plt.plot(t_, theta1_, linewidth=2)
plt.ylabel(r"$\theta_1$ [rad]")
plt.xlim(0, t_total)

plt.subplot(3,1,2)
plt.plot(t_, dtheta1_, linewidth=2)
plt.ylabel(r"$\omega_1$ [rad/s]")
plt.xlim(0, t_total)

plt.subplot(3,1,3)
plt.plot(t_, ddtheta1_, linewidth=2)
plt.ylabel(r"$\alpha_1$ [rad/s²]")
plt.xlabel(r"$t$ [s]")
plt.xlim(0, t_total)

plt.tight_layout()

plt.figure()

plt.subplot(3,1,1)
plt.plot(t_, theta2_, linewidth=2)
plt.ylabel(r"$\theta_2$ [rad]")
plt.xlim(0, t_total)

plt.subplot(3,1,2)
plt.plot(t_, dtheta2_, linewidth=2)
plt.ylabel(r"$\omega_2$ [rad/s]")
plt.xlim(0, t_total)

plt.subplot(3,1,3)
plt.plot(t_, ddtheta2_, linewidth=2)
plt.ylabel(r"$\alpha_2$ [rad/s²]")
plt.xlabel(r"$t$ [s]")
plt.xlim(0, t_total)

plt.tight_layout()


# =========================
# CINEMÀTICA DIRECTA XY
# =========================
i_t = i / rG

x = (0*theta1_ - theta2_) / i_t
y = (theta1_ + theta2_) / i_t

plt.figure()
plt.plot(x, y, linewidth=2)
plt.plot([0,x_f],[0,y_f],'o')
plt.xlabel(r"$x$ [m]")
plt.ylabel(r"$y$ [m]")
plt.axis("equal")


# =========================
# XY vs temps
# =========================
plt.figure()

plt.subplot(3,1,1)
plt.plot(t_, x, linewidth=2)
plt.ylabel(r"$x$ [m]")
plt.xlim(0, t_total)

plt.subplot(3,1,2)
plt.plot(t_, np.gradient(x, t_), linewidth=2)
plt.ylabel(r"$\dot{x}$ [m/s]")
plt.xlim(0, t_total)

plt.subplot(3,1,3)
plt.plot(t_, np.gradient(np.gradient(x, t_), t_), linewidth=2)
plt.ylabel(r"$\ddot{x}$ [m/s²]]")
plt.xlabel(r"$t$ [s]")
plt.xlim(0, t_total)

plt.tight_layout()

plt.show()