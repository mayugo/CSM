import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

# =========================================================
# PARÀMETRES
# =========================================================

# Motor
n_max = 3600    # rpm, velocitat maxima
Mm_n  = 3.53    # Nm, parell motor nominal
Mm_max= 12.2    # Nm, parell motor maxim 
J_m   = 260e-6  # kg·m2, inercia del motor
b_m   = 1e-3    # Nms/rad, constant motor

# Cargol
pas = 10e-3     # m, pas cargol
D   = 23.2e-3   # m, diàmetre cargol
L_c = 940e-3    # m, longitud
rho = 7.85e3    # kg/m3, densitat acer

# Acoblament
J_a = 30e-6     # kg/m2, inercia de l'acoblament

# Massa
m_carro = 1.35  # kg, massa del carro
M = 0           # kg, massa addicional

# %%
# =========================================================
# CARGOL
# =========================================================

v_cargol = np.pi * D**2 / 4 * L_c
m_cargol = v_cargol * rho
J_cargol = 0.5 * m_cargol * (D/2)**2

i_n = pas / (2*np.pi)

# =========================================================
# MOTOR DC
# =========================================================

w_m_n = (3600/30)*np.pi * (12.2 - 3.53)/12.2
M_m_n = 3.53
V_m_n = 120

A_m_n = (w_m_n * M_m_n) / V_m_n

K_m = M_m_n / A_m_n   # Nm / A
K_b = V_m_n / w_m_n

L_e = 0.82e-3
R = 1.6         # ohms

# =========================================================
# DINÀMICA MECÀNICA
# =========================================================

J = J_m + J_a + J_cargol + (m_carro + M) * i_n**2
b = b_m
K = K_m

# =========================================================
# FUNCIÓ TRANSFERÈNCIA MOTOR DC
# =========================================================

s = ctrl.TransferFunction.s

model_M = K / ((J*s + b)*(L_e*s + R) + K**2)

print(model_M)

# %%
# BODE log - log
# -----------------------------
plt.figure()
ctrl.bode(model_M, dB=True)
plt.suptitle("Diagrama de Bode log-log")

# BODE lineal - linesl
# -----------------------------
omega = np.linspace(0, 150, 1000)
resp = ctrl.frequency_response(model_M, omega)

plt.figure()

plt.subplot(2,1,1)
plt.plot(omega, resp.magnitude)
plt.ylabel("Magnitud (lineal)")
plt.grid()

plt.subplot(2,1,2)
plt.plot(omega, resp.phase)
plt.ylabel("Fase (rad)")
plt.xlabel("Freq (rad/s)")
plt.grid()

plt.suptitle("Diagrama de Bode lin-lin")
plt.tight_layout()
plt.show()

# %%
# =========================================================
# STEP RESPONSE
# =========================================================
u = 120     # V, voltatge

t, y = ctrl.step_response(u * model_M)

plt.figure()
plt.plot(t, y)
plt.grid()
plt.xlabel("temps (s)")
plt.ylabel("velocitat (rad/s)")
plt.title("Resposta escaló")

# %%
# =========================================================
# INPUT RECTANGULAR (0 → 0.03 s)
# =========================================================
t_f = 0.06
t_a = 0.03
t = np.linspace(0, t_f, 20000)

V= np.zeros_like(t)
V[ t <= t_a] = u   # Moment motor, actua només fins 3 segons


t, y = ctrl.forced_response(model_M, T=t, U=V)

plt.figure(figsize=(6,8))
plt.subplot(3,1,1)
plt.plot(t, V)
plt.ylabel("Volt (V)")
plt.grid()

plt.subplot(3,1,2)
plt.plot(t, y * 30/np.pi)
plt.ylabel("rpm")
plt.grid()

plt.subplot(3,1,3)
plt.plot(t, y * i_n)
plt.ylabel("m/s")
plt.xlabel("temps (s)")
plt.grid()

plt.tight_layout()
plt.show()

# %%
# =========================================================
# PWM (100 polsos)
# =========================================================
t = np.linspace(0, t_f, 20000)

Npulses = 16
Duty = 0.8

V = np.zeros_like(t)

# =========================================================
# PWM només dins [0, t_a]
# =========================================================
mask = t <= t_a
t_pwm = t[mask]

Tcycle = t_a / Npulses
Ton = Duty * Tcycle

for k in range(Npulses):
    t_start = k * Tcycle
    t_end = t_start + Ton
    V[mask & (t >= t_start) & (t < t_end)] = u

t, y = ctrl.forced_response(model_M, T=t, U=V)

plt.figure(figsize=(6,8))
plt.subplot(3,1,1)
plt.plot(t, V)
plt.ylabel("Volt (V)")
plt.grid()

plt.subplot(3,1,2)
plt.plot(t, y * 30/np.pi)
plt.ylabel("rpm")
plt.grid()

plt.subplot(3,1,3)
plt.plot(t, y * i_n)
plt.ylabel("m/s")
plt.xlabel("temps (s)")
plt.grid()

plt.tight_layout()
plt.show()