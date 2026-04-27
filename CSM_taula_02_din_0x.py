import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# =========================================================
# DADES (equivalent MATLAB)
# =========================================================

# motor
n_nom  = 1470      # rpm
Pot_nom = 1600     # W
mmot    = 6        # kg
Jmot    = 0.01     # kg·m2

# reductor 40:1
i       = 40
mred    = 54
Jred    = 1.2
Jsor    = 2

# sistema mecànic
Jtau    = 10
Npec    = 4
mpec    = 12
R       = 0.5

# fregament
F1 = 100
F2 = 50   # F = F1 + F2*omega

# =========================================================
# PARÀMETRES EQUIVALENTS
# =========================================================

Jeq = Jmot + (Jsor + Jtau)/(i*i) + Npec*mpec*R*R/(i*i)
beq = F2 * R / (i*i)
M_r_eq = F1 * R / i
M_m = Pot_nom / (n_nom*np.pi/30)

# =========================================================
# MODELS MISO (G i H)
# =========================================================

num = [1]
num_neg = [-1]

den_G = [Jeq, beq]
den_H = [Jeq, beq, 0]

G1 = signal.lti(num, den_G)
G2 = signal.lti(num_neg, den_G)

H1 = signal.lti(num, den_H)
H2 = signal.lti(num_neg, den_H)

# =========================================================
# TEMPS
# =========================================================

t_f = 20
dt = 1e-3
t = np.arange(0, t_f, dt)

# =========================================================
# ENTRADES
# =========================================================

M_m_ = M_m * np.ones_like(t)
M_r_eq_ = M_r_eq * np.ones_like(t)

# =========================================================
# RETARD (equivalent exp(-10s))
# =========================================================

def delay_signal(u, t, delay):
    dt = t[1] - t[0]
    shift = int(delay / dt)
    return np.concatenate((np.zeros(shift), u[:-shift]))

M_m_delayed = delay_signal(M_m_, t, 10)

# =========================================================
# SIMULACIONS
# =========================================================

_, outG_1, _ = signal.lsim(G1, M_m_delayed, t)
_, outG_2, _ = signal.lsim(G2, M_r_eq_, t)

_, outH_1, _ = signal.lsim(H1, M_m_delayed, t)
_, outH_2, _ = signal.lsim(H2, M_r_eq_, t)

# superposició (MISO equivalent)
_, outG_T, _ = signal.lsim(G1, M_m_delayed - M_r_eq_, t)
_, outH_T, _ = signal.lsim(H1, M_m_delayed - M_r_eq_, t)

# =========================================================
# PLOTS
# =========================================================

plt.figure(figsize=(10,7))

# --- motor
plt.subplot(3,2,1)
plt.plot(t, outG_1, linewidth=1.2)
plt.ylabel('velocitat [rad/s]')
plt.title('Resposta motor')
plt.grid()

plt.subplot(3,2,2)
plt.plot(t, outH_1, linewidth=1.2)
plt.ylabel('gir [rad]')
plt.title('Resposta motor')
plt.grid()

# --- fregament
plt.subplot(3,2,3)
plt.plot(t, outG_2, linewidth=1.2)
plt.ylabel('velocitat [rad/s]')
plt.title('Resposta fregament')
plt.grid()

plt.subplot(3,2,4)
plt.plot(t, outH_2, linewidth=1.2)
plt.ylabel('gir [rad]')
plt.title('Resposta fregament')
plt.grid()

# --- superposició velocitat
plt.subplot(3,2,5)
plt.plot(t, np.maximum(0, outG_1 + outG_2), linewidth=1.2)
plt.plot(t, outG_T, linewidth=1.2)
plt.legend(['Superposició', 'SISO equivalent'], frameon=False)
plt.ylabel('velocitat [rad/s]')
plt.xlabel('temps [s]')
plt.grid()

# --- superposició gir
plt.subplot(3,2,6)
sumH = outH_1 + outH_2
plt.plot(t, sumH - np.min(sumH), linewidth=1.2)
plt.plot(t, outH_T, linewidth=1.2)
plt.legend(['Superposició', 'SISO equivalent'], frameon=False)
plt.ylabel('gir [rad]')
plt.xlabel('temps [s]')
plt.grid()

plt.tight_layout()
plt.show()