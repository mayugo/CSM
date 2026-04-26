import numpy as np
import matplotlib.pyplot as plt

# --- DADES ---
W = 0.112
R = 0.028
D = 0.045
i_r = 10

a_max = 0.1
tt = 20.0
x = 2.5

ddphi_max = 60.0

x22, x33, x44 = 0.8, 1.2, 2.0
phi1, phi2, phi3, phi4, phi5 = 0, 90, -45, 0, 0

# --- TEMPS ---
t_ = np.arange(0, tt, 1/20)

# --- TRAJECTÒRIA IDEAL ---
x_ideal_ = x/tt * t_

phi_ideal_ = np.zeros_like(t_)

for i, xi in enumerate(x_ideal_):
    if xi <= x22:
        phi_ideal_[i] = phi1 + (phi2-phi1)*xi/x22
    elif xi <= x33:
        phi_ideal_[i] = phi2 + (phi3-phi2)*(xi-x22)/(x33-x22)
    elif xi <= x44:
        phi_ideal_[i] = phi3 + (phi4-phi3)*(xi-x33)/(x44-x33)
    else:
        phi_ideal_[i] = phi4 + (phi5-phi4)*(xi-x44)/(x-x44)

# --- PERFIL TRAPEZOIDAL ---
t_b = tt/2 - np.sqrt(a_max**2*tt**2 - 4*a_max*x)/(2*a_max)
v_max = a_max * t_b
x_b = 0.5*a_max*t_b**2

x_ = np.zeros_like(t_)
v_ = np.zeros_like(t_)
a_ = np.zeros_like(t_)

for i, t in enumerate(t_):
    if t <= t_b:
        x_[i] = 0.5*a_max*t**2
        v_[i] = a_max*t
        a_[i] = a_max
    elif t <= tt - t_b:
        x_[i] = x_b + v_max*(t - t_b)
        v_[i] = v_max
        a_[i] = 0
    else:
        dt = t - (tt - t_b)
        x_[i] = x - 0.5*a_max*(t_b - dt)**2
        v_[i] = v_max - a_max*dt
        a_[i] = -a_max

# --- GIR (SIMPLIFICAT: reutilitzem ideal) ---
phi_ = phi_ideal_.copy()
dphi_ = np.gradient(phi_, t_)
ddphi_ = np.gradient(dphi_, t_)

# --- CINEMÀTICA INVERSA ---
phi_rad = np.deg2rad(phi_)

theta_r = 1/R * x_ + W/(2*R)*phi_rad
theta_l = 1/R * x_ - W/(2*R)*phi_rad

dtheta_r = 1/R * v_ + W/(2*R)*np.deg2rad(dphi_)
dtheta_l = 1/R * v_ - W/(2*R)*np.deg2rad(dphi_)

ddtheta_r = 1/R * a_ + W/(2*R)*np.deg2rad(ddphi_)
ddtheta_l = 1/R * a_ - W/(2*R)*np.deg2rad(ddphi_)

# Motors
theta_m_r = i_r * theta_r
theta_m_l = i_r * theta_l

# --- RECONSTRUCCIÓ TRAJECTÒRIA ---
p_x = np.zeros_like(t_)
p_y = np.zeros_like(t_)

for i in range(1, len(t_)):
    dx = x_[i] - x_[i-1]
    dphi = np.deg2rad(phi_[i] - phi_[i-1])

    if abs(dphi) < 1e-6:
        C = dx
    else:
        C = 2*np.sin(dphi/2)*dx/dphi

    p_x[i] = p_x[i-1] + C*np.cos(np.deg2rad(phi_[i]))
    p_y[i] = p_y[i-1] + C*np.sin(np.deg2rad(phi_[i]))

# --- PLOTS ---

# Posició, velocitat, acceleració
fig, axs = plt.subplots(3,1, figsize=(8,8))

axs[0].plot(t_, x_, color='tab:blue')
axs[0].set_ylabel("x [m]")

axs[1].plot(t_, v_, color='tab:green')
axs[1].set_ylabel("v [m/s]")

axs[2].plot(t_, a_, color='tab:red')
axs[2].set_ylabel("a [m/s²]")
axs[2].set_xlabel("t [s]")

for ax in axs:
    ax.grid()

plt.tight_layout()

# Trajectòria XY
plt.figure()
plt.plot(p_x, p_y, color='#D95319')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.axis('equal')
plt.grid()

plt.show()