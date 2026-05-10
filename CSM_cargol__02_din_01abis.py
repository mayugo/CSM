import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

plt.rcParams.update({"font.size": 14})

# =========================================================
# DADES
# =========================================================

n_max = 3600
Mm_n  = 3.53
Mm_max= 12.2
J_m   = 260e-6
b_m   = 1e-3

pas = 10e-3
D   = 23.2e-3
L   = 940e-3
rho = 7.85e3

J_a = 30e-6
k_a = 120e-3

m_carro = 1.35
M = 0

# =========================================================
# GEOMETRIA CARGOL
# =========================================================

v_cargol = np.pi * D**2 / 4 * L
m_cargol = v_cargol * rho
J_cargol = 0.5 * m_cargol * (D/2)**2

i_n = pas / (2*np.pi)

# =========================================================
# MATRIUS M,C,K
# =========================================================

J11 = J_m + J_a/2
J22 = (J_a/2 + J_cargol)/i_n**2 + m_carro + M

C11 = b_m

K11 = k_a
K12 = -k_a/i_n
K22 = k_a/i_n**2

M_ = np.array([[J11, 0],
               [0,   J22]])

C_ = np.array([[C11, 0],
               [0,   0]])

K_ = np.array([[K11, K12],
               [K12, K22]])

# =========================================================
# ESPAI D'ESTAT
# x = [q1, q2, w1, w2]
# =========================================================

Z = np.zeros((2,2))
I = np.eye(2)

A = np.block([
    [Z, I],
    [-np.linalg.inv(M_) @ K_, -np.linalg.inv(M_) @ C_]
])

B = np.block([
    [np.zeros((2,2))],
    [np.linalg.inv(M_)]
])

C = np.eye(4)
D = np.zeros((4,2))

# =========================================================
# MODEL STATE SPACE SCIpy
# =========================================================

sys = signal.StateSpace(A, B, C, D)

# =========================================================
# SIMULACIÓ
# =========================================================

t_final = 6
t = np.linspace(0, t_final, 20000)

Mm = Mm_n / 10

u = np.zeros((2, len(t)))
u[0, t <= 3] = Mm     # motor actiu fins 3 s
u[1, :] = 0

t, y, x = signal.lsim(sys, U=u.T, T=t)

# =========================================================
# SORTIDES
# =========================================================

q1 = y[:,0]
q2 = y[:,1]
w1 = y[:,2]
w2 = y[:,3]

# =========================================================
# ENTRADA
# =========================================================

u_plot = u[0]

# =========================================================
# PLOTS
# =========================================================

plt.figure(figsize=(6,8))

# --- entrada
plt.subplot(3,1,1)
plt.plot(t, u_plot, linewidth=1.5)
plt.ylabel("Parell motor (Nm)")
plt.grid()

# --- velocitats
plt.subplot(3,1,2)
plt.plot(t, w1*30/np.pi, label="eix motor")
plt.plot(t, w2*30/np.pi/i_n, label="cargol")
plt.ylabel("Velocitat (rpm)")
plt.legend()
plt.grid()

# --- desplaçament
plt.subplot(3,1,3)
plt.plot(t, q2, label="carro")
plt.ylabel("Desplaçament (m)")
plt.xlabel("temps (s)")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()